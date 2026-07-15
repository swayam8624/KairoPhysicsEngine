module;

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Buoyancy;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.World;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;
    using namespace kairo::foundation::geometry;

    using WaterVolumeID = std::uint32_t;
    inline constexpr WaterVolumeID InvalidWaterVolumeID =
        std::numeric_limits<WaterVolumeID>::max();

    /// Axis-aligned gameplay water volume. Full SPH/FLIP fluid remains a later
    /// specialized system; this is the useful rigid-body interaction layer.
    struct WaterVolumeDesc final
    {
        AABBf Bounds = AABBf::FromMinMax(
            Vec3f{ -5.0f, -1.0f, -5.0f },
            Vec3f{ 5.0f, 1.0f, 5.0f });
        float Density = 1.0f;
        float LinearDrag = 1.0f;
        float AngularDrag = 0.25f;
        std::uint32_t Layer = CollisionLayer::Fluid;
    };

    /// Per-body displaced-volume data. This is explicit because a rigid body's
    /// mass does not uniquely define its physical volume or density.
    struct BuoyancyBodyDesc final
    {
        float DisplacedVolume = 1.0f;
        float BuoyancyScale = 1.0f;
        float LinearDragScale = 1.0f;
        float AngularDragScale = 1.0f;
    };

    enum class WaterVolumeEventType : std::uint8_t
    {
        Enter,
        Stay,
        Exit
    };

    struct WaterVolumeEvent final
    {
        WaterVolumeID Volume = InvalidWaterVolumeID;
        BodyID Body = InvalidBodyID;
        WaterVolumeEventType Type = WaterVolumeEventType::Enter;
        float Submersion = 0.0f;
    };

    inline void ValidateWaterVolumeDesc(const WaterVolumeDesc& desc)
    {
        if (!desc.Bounds.IsValid())
        {
            throw std::invalid_argument("WaterVolumeDesc.Bounds must be a valid finite AABB.");
        }
        RequirePositive(desc.Density, "WaterVolumeDesc.Density");
        RequireNonNegative(desc.LinearDrag, "WaterVolumeDesc.LinearDrag");
        RequireNonNegative(desc.AngularDrag, "WaterVolumeDesc.AngularDrag");
        if (desc.Layer == 0u)
        {
            throw std::invalid_argument("WaterVolumeDesc.Layer must contain at least one collision layer bit.");
        }
    }

    inline void ValidateBuoyancyBodyDesc(const BuoyancyBodyDesc& desc)
    {
        RequirePositive(desc.DisplacedVolume, "BuoyancyBodyDesc.DisplacedVolume");
        RequireNonNegative(desc.BuoyancyScale, "BuoyancyBodyDesc.BuoyancyScale");
        RequireNonNegative(desc.LinearDragScale, "BuoyancyBodyDesc.LinearDragScale");
        RequireNonNegative(desc.AngularDragScale, "BuoyancyBodyDesc.AngularDragScale");
    }

    struct WaterVolumeState final
    {
        WaterVolumeID ID = InvalidWaterVolumeID;
        bool Active = false;
        WaterVolumeDesc Desc;
    };

    struct BuoyancyBodyState final
    {
        BodyID Body = InvalidBodyID;
        bool Active = false;
        BuoyancyBodyDesc Desc;
    };

    /// Rigid-body water interaction subsystem.
    ///
    /// Input: registered dynamic bodies and axis-aligned water volumes.
    /// Output: buoyancy/drag forces applied before the caller steps PhysicsWorld,
    /// plus deterministic enter/stay/exit events.
    /// Task: establish a real gameplay-water foundation while deliberately
    /// keeping particle-fluid simulation out of the rigid-body engine.
    class BuoyancySystem final
    {
    public:
        using EventCallback = std::function<void(const WaterVolumeEvent&)>;

        [[nodiscard]]
        WaterVolumeID AddWaterVolume(const WaterVolumeDesc& desc)
        {
            ValidateWaterVolumeDesc(desc);
            const WaterVolumeID id = static_cast<WaterVolumeID>(m_Volumes.size());
            m_Volumes.push_back({ id, true, desc });
            return id;
        }

        void RemoveWaterVolume(WaterVolumeID volume)
        {
            if (volume >= m_Volumes.size())
            {
                throw std::out_of_range("BuoyancySystem::RemoveWaterVolume failed: volume id does not exist.");
            }
            m_Volumes.at(volume).Active = false;
        }

        void RegisterBody(BodyID body, const BuoyancyBodyDesc& desc = {})
        {
            ValidateBuoyancyBodyDesc(desc);
            if (body >= m_Bodies.size())
            {
                m_Bodies.resize(static_cast<std::size_t>(body) + 1u);
            }
            m_Bodies.at(body) = { body, true, desc };
        }

        void UnregisterBody(BodyID body)
        {
            if (body < m_Bodies.size())
            {
                m_Bodies.at(body).Active = false;
            }
        }

        [[nodiscard]]
        const std::vector<WaterVolumeEvent>& LastEvents() const noexcept
        {
            return m_LastEvents;
        }

        [[nodiscard]]
        const std::vector<WaterVolumeState>& Volumes() const noexcept
        {
            return m_Volumes;
        }

        void SetEventCallback(EventCallback callback)
        {
            m_EventCallback = std::move(callback);
        }

        /// Input: world state before its next `PhysicsWorld::Step` and a valid dt.
        /// Output: force accumulators updated and water lifecycle events emitted.
        /// Task: estimate vertical submersion from the body's finite collider
        /// bounds, apply Archimedes buoyancy opposite gravity, and apply linear/
        /// angular drag proportional to submersion. Bodies without finite shapes
        /// are ignored because their displaced area cannot be inferred here.
        void Step(PhysicsWorld& world, float dt)
        {
            RequirePositive(dt, "BuoyancySystem.dt");
            m_LastEvents.clear();
            std::vector<OverlapKey> current;

            for (const BuoyancyBodyState& registered : m_Bodies)
            {
                if (!registered.Active || !world.IsValidBody(registered.Body))
                {
                    continue;
                }

                const RigidBody& body = world.Bodies().at(registered.Body);
                if (!IsDynamic(body))
                {
                    continue;
                }

                const auto bodyBounds = ComputeBodyBounds(world, registered.Body);
                if (!bodyBounds)
                {
                    continue;
                }

                for (const WaterVolumeState& volume : m_Volumes)
                {
                    if (!volume.Active || !Intersects(*bodyBounds, volume.Desc.Bounds))
                    {
                        continue;
                    }

                    const float submersion = ComputeSubmersion(*bodyBounds, volume.Desc.Bounds);
                    if (submersion <= 0.0f)
                    {
                        continue;
                    }

                    current.push_back({ volume.ID, registered.Body });
                    ApplyForces(world, registered, volume, submersion);
                    EmitLifecycle(volume.ID, registered.Body, submersion);
                }
            }

            EmitExited(current);
            m_PreviousOverlaps = std::move(current);
        }

    private:
        struct OverlapKey final
        {
            WaterVolumeID Volume = InvalidWaterVolumeID;
            BodyID Body = InvalidBodyID;

            [[nodiscard]] friend constexpr bool operator==(const OverlapKey&, const OverlapKey&) noexcept = default;
        };

        std::vector<WaterVolumeState> m_Volumes;
        std::vector<BuoyancyBodyState> m_Bodies;
        std::vector<OverlapKey> m_PreviousOverlaps;
        std::vector<WaterVolumeEvent> m_LastEvents;
        EventCallback m_EventCallback;

        [[nodiscard]]
        static std::optional<AABBf> ComputeBodyBounds(const PhysicsWorld& world, BodyID body)
        {
            AABBf result = AABBf::Empty();
            bool hasBounds = false;
            for (const Collider& collider : world.Colliders())
            {
                if (!IsActiveCollider(collider) || collider.Body != body || IsInfiniteCollider(collider))
                {
                    continue;
                }
                const AABBf bounds = WorldAABB(world.Bodies().at(body), collider);
                if (!hasBounds)
                {
                    result = bounds;
                    hasBounds = true;
                }
                else
                {
                    result.ExpandToInclude(bounds);
                }
            }
            return hasBounds ? std::optional<AABBf>{ result } : std::nullopt;
        }

        [[nodiscard]]
        static float ComputeSubmersion(const AABBf& body, const AABBf& water)
        {
            const float height = body.Max.y - body.Min.y;
            if (height <= 1.0e-8f)
            {
                return 0.0f;
            }
            const float submergedHeight = std::clamp(
                std::min(body.Max.y, water.Max.y) - std::max(body.Min.y, water.Min.y),
                0.0f,
                height);
            return submergedHeight / height;
        }

        void ApplyForces(
            PhysicsWorld& world,
            const BuoyancyBodyState& registered,
            const WaterVolumeState& volume,
            float submersion)
        {
            const RigidBody& body = world.Bodies().at(registered.Body);
            const float gravityMagnitude = world.Gravity.Length();
            if (gravityMagnitude > 1.0e-8f)
            {
                const Vec3f up = -world.Gravity / gravityMagnitude;
                const float magnitude = volume.Desc.Density * registered.Desc.DisplacedVolume *
                    gravityMagnitude * submersion * registered.Desc.BuoyancyScale;
                world.AddBodyForce(registered.Body, up * magnitude);
            }

            world.AddBodyForce(
                registered.Body,
                -body.State.LinearVelocity * volume.Desc.LinearDrag *
                    registered.Desc.LinearDragScale * submersion);
            world.AddBodyTorque(
                registered.Body,
                -body.State.AngularVelocity * volume.Desc.AngularDrag *
                    registered.Desc.AngularDragScale * submersion);
        }

        void Emit(const WaterVolumeEvent& event)
        {
            m_LastEvents.push_back(event);
            if (m_EventCallback)
            {
                m_EventCallback(event);
            }
        }

        void EmitLifecycle(WaterVolumeID volume, BodyID body, float submersion)
        {
            const OverlapKey key{ volume, body };
            const bool existed = std::find(m_PreviousOverlaps.begin(), m_PreviousOverlaps.end(), key) !=
                m_PreviousOverlaps.end();
            Emit({ volume, body, existed ? WaterVolumeEventType::Stay : WaterVolumeEventType::Enter, submersion });
        }

        void EmitExited(const std::vector<OverlapKey>& current)
        {
            for (const OverlapKey& prior : m_PreviousOverlaps)
            {
                if (std::find(current.begin(), current.end(), prior) == current.end())
                {
                    Emit({ prior.Volume, prior.Body, WaterVolumeEventType::Exit, 0.0f });
                }
            }
        }
    };
}
