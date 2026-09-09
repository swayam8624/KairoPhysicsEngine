module;

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>

export module Kairo.Foundation.PhysicsEngine.CharacterController;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.World;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    /// Query-driven upright character shape. Position is the center of the
    /// complete capsule. The straight center segment has length
    /// `Height - 2 * Radius`; SweepSamples places sphere queries along that
    /// segment to approximate a swept capsule without introducing a second
    /// collision/narrowphase implementation beside PhysicsWorld.
    struct CharacterControllerConfig final
    {
        float Radius = 0.35f;
        float Height = 1.80f;
        float SkinWidth = 0.02f;
        float GroundProbeDistance = 0.12f;
        float MaxSlopeAngleRadians = 0.872664626f; // 50 degrees.
        std::uint32_t MaxSlideIterations = 5u;
        std::uint32_t SweepSamples = 5u;
        std::uint32_t CollisionMask = CollisionLayer::All;
        bool EnableGroundSnap = true;

        void Validate() const
        {
            RequirePositive(Radius, "CharacterController.Radius");
            RequirePositive(Height, "CharacterController.Height");
            RequireNonNegative(SkinWidth, "CharacterController.SkinWidth");
            RequireNonNegative(GroundProbeDistance,
                "CharacterController.GroundProbeDistance");
            RequireFinite(MaxSlopeAngleRadians,
                "CharacterController.MaxSlopeAngleRadians");
            if (Height < Radius * 2.0f)
                throw std::invalid_argument(
                    "Character controller height must be at least two radii.");
            if (SkinWidth >= Radius)
                throw std::invalid_argument(
                    "Character controller skin width must be smaller than its radius.");
            if (MaxSlopeAngleRadians < 0.0f ||
                MaxSlopeAngleRadians >= Pi * 0.5f)
                throw std::invalid_argument(
                    "Character controller maximum slope angle must be in [0, pi/2).");
            if (MaxSlideIterations == 0u || MaxSlideIterations > 16u)
                throw std::invalid_argument(
                    "Character controller slide iterations must be within 1..16.");
            if (SweepSamples == 0u || SweepSamples > 17u)
                throw std::invalid_argument(
                    "Character controller sweep samples must be within 1..17.");
            if (CollisionMask == 0u)
                throw std::invalid_argument(
                    "Character controller collision mask cannot be empty.");
        }
    };

    struct CharacterGroundState final
    {
        bool Grounded = false;
        BodyID Body = InvalidBodyID;
        ColliderID Collider = InvalidColliderID;
        Vec3f Point = Vec3f::Zero();
        Vec3f Normal = Vec3f::Up();
        float Distance = 0.0f;
    };

    struct CharacterMoveResult final
    {
        Vec3f RequestedDisplacement = Vec3f::Zero();
        Vec3f AppliedDisplacement = Vec3f::Zero();
        Vec3f RemainingDisplacement = Vec3f::Zero();
        CharacterGroundState Ground;
        bool HitWall = false;
        bool HitCeiling = false;
        std::uint32_t SlideIterations = 0u;
    };

    class KinematicCharacterController final
    {
    public:
        explicit KinematicCharacterController(
            CharacterControllerConfig config = {},
            const Vec3f& position = Vec3f::Zero())
            : m_Config(config), m_Position(position)
        {
            m_Config.Validate();
            RequireFinite(m_Position, "CharacterController.Position");
        }

        [[nodiscard]] const CharacterControllerConfig& Config() const noexcept
        {
            return m_Config;
        }

        [[nodiscard]] const Vec3f& Position() const noexcept
        {
            return m_Position;
        }

        [[nodiscard]] const CharacterGroundState& GroundState() const noexcept
        {
            return m_Ground;
        }

        void Teleport(const Vec3f& position)
        {
            RequireFinite(position, "CharacterController.Teleport");
            m_Position = position;
            m_Ground = {};
        }

        /// Sweeps the virtual capsule, advances to the contact skin, and
        /// iteratively projects the unconsumed displacement onto blocking
        /// surfaces. Walkable slopes count as ground; steep slopes remain walls.
        /// Ground snap is query-only and never mutates PhysicsWorld rigid bodies.
        [[nodiscard]] CharacterMoveResult Move(
            const PhysicsWorld& world,
            const Vec3f& desiredDisplacement,
            ColliderID ignoredCollider = InvalidColliderID)
        {
            RequireFinite(desiredDisplacement,
                "CharacterController.DesiredDisplacement");

            CharacterMoveResult result;
            result.RequestedDisplacement = desiredDisplacement;
            Vec3f remaining = desiredDisplacement;
            const float slopeCosine = std::cos(m_Config.MaxSlopeAngleRadians);
            constexpr float movementEpsilon = 1.0e-6f;

            for (std::uint32_t iteration = 0u;
                iteration < m_Config.MaxSlideIterations;
                ++iteration)
            {
                const float remainingLength = remaining.Length();
                if (!std::isfinite(remainingLength) ||
                    remainingLength <= movementEpsilon)
                    break;

                const auto hit = SweepVirtualCapsule(
                    world, m_Position, remaining, ignoredCollider);
                if (!hit.has_value())
                {
                    m_Position += remaining;
                    result.AppliedDisplacement += remaining;
                    remaining = Vec3f::Zero();
                    break;
                }

                ++result.SlideIterations;
                const Vec3f direction = remaining / remainingLength;
                const float advanceDistance = std::clamp(
                    hit->Distance - m_Config.SkinWidth,
                    0.0f,
                    remainingLength);
                const Vec3f advance = direction * advanceDistance;
                m_Position += advance;
                result.AppliedDisplacement += advance;

                Vec3f unconsumed = remaining - advance;
                const Vec3f normal = SafeNormal(hit->Normal);
                const float verticalNormal = normal.y;
                if (verticalNormal >= slopeCosine)
                {
                    result.Ground = GroundFromHit(*hit);
                    result.Ground.Grounded = true;
                }
                else if (verticalNormal <= -0.05f)
                {
                    result.HitCeiling = true;
                }
                else
                {
                    result.HitWall = true;
                }

                const float intoSurface = Dot(unconsumed, normal);
                if (intoSurface < 0.0f)
                    unconsumed -= normal * intoSurface;

                // A zero-distance hit whose projection cannot materially alter
                // the remaining motion would otherwise spin through every
                // iteration on an initial overlap/tangent numerical contact.
                if (advanceDistance <= movementEpsilon &&
                    (unconsumed - remaining).LengthSquared() <=
                        movementEpsilon * movementEpsilon)
                {
                    remaining = Vec3f::Zero();
                    break;
                }
                remaining = unconsumed;
            }

            result.RemainingDisplacement = remaining;

            // Refresh grounding after the final slide. Ground snap deliberately
            // runs only when the requested motion is not upward, so jumping is
            // never pulled back to the floor by the convenience probe.
            m_Ground = ProbeGround(world, ignoredCollider);
            if (m_Config.EnableGroundSnap &&
                desiredDisplacement.y <= movementEpsilon &&
                m_Ground.Grounded)
            {
                const float snapDistance = std::max(
                    0.0f, m_Ground.Distance - m_Config.SkinWidth);
                if (snapDistance <= m_Config.GroundProbeDistance)
                {
                    const Vec3f snap{ 0.0f, -snapDistance, 0.0f };
                    m_Position += snap;
                    result.AppliedDisplacement += snap;
                    m_Ground.Distance = m_Config.SkinWidth;
                }
            }
            result.Ground = m_Ground;
            return result;
        }

        [[nodiscard]] CharacterGroundState ProbeGround(
            const PhysicsWorld& world,
            ColliderID ignoredCollider = InvalidColliderID) const
        {
            CharacterGroundState ground;
            const float probeDistance =
                m_Config.GroundProbeDistance + m_Config.SkinWidth;
            if (probeDistance <= 1.0e-8f) return ground;

            const auto hit = SweepVirtualCapsule(
                world,
                m_Position,
                Vec3f{ 0.0f, -probeDistance, 0.0f },
                ignoredCollider);
            if (!hit.has_value()) return ground;

            const Vec3f normal = SafeNormal(hit->Normal);
            if (normal.y < std::cos(m_Config.MaxSlopeAngleRadians))
                return ground;
            ground = GroundFromHit(*hit);
            ground.Normal = normal;
            ground.Grounded = true;
            return ground;
        }

    private:
        CharacterControllerConfig m_Config;
        Vec3f m_Position = Vec3f::Zero();
        CharacterGroundState m_Ground;

        [[nodiscard]] static float Dot(const Vec3f& a, const Vec3f& b) noexcept
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        [[nodiscard]] static Vec3f SafeNormal(const Vec3f& normal) noexcept
        {
            const float length = normal.Length();
            return length > 1.0e-8f && std::isfinite(length)
                ? normal / length
                : Vec3f::Up();
        }

        [[nodiscard]] CharacterGroundState GroundFromHit(
            const PhysicsSweepHit& hit) const noexcept
        {
            CharacterGroundState ground;
            ground.Body = hit.Body;
            ground.Collider = hit.Collider;
            ground.Point = hit.Point;
            ground.Normal = hit.Normal;
            ground.Distance = hit.Distance;
            return ground;
        }

        [[nodiscard]] float CapsuleHalfSegment() const noexcept
        {
            return std::max(0.0f, m_Config.Height * 0.5f - m_Config.Radius);
        }

        [[nodiscard]] std::optional<PhysicsSweepHit> SweepVirtualCapsule(
            const PhysicsWorld& world,
            const Vec3f& center,
            const Vec3f& displacement,
            ColliderID ignoredCollider) const
        {
            const float travel = displacement.Length();
            if (!std::isfinite(travel) || travel <= 1.0e-8f)
                return std::nullopt;

            const float halfSegment = CapsuleHalfSegment();
            const std::uint32_t sampleCount =
                halfSegment <= 1.0e-8f ? 1u : m_Config.SweepSamples;
            std::optional<PhysicsSweepHit> closest;

            for (std::uint32_t sample = 0u; sample < sampleCount; ++sample)
            {
                const float alpha = sampleCount == 1u
                    ? 0.5f
                    : static_cast<float>(sample) /
                        static_cast<float>(sampleCount - 1u);
                const float offsetY = -halfSegment +
                    (halfSegment * 2.0f) * alpha;
                const auto hit = world.SweepSphere(
                    center + Vec3f{ 0.0f, offsetY, 0.0f },
                    displacement,
                    m_Config.Radius,
                    m_Config.CollisionMask,
                    ignoredCollider);
                if (hit.has_value() &&
                    (!closest.has_value() || hit->Distance < closest->Distance))
                    closest = hit;
            }
            return closest;
        }
    };
}
