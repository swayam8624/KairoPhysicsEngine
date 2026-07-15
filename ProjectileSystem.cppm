module;

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Projectile;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.World;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    using ProjectileID = std::uint32_t;
    inline constexpr ProjectileID InvalidProjectileID =
        std::numeric_limits<ProjectileID>::max();

    /// Hitscan has no persistent travel time. Ballistic projectiles advance
    /// through time and use `PhysicsWorld::SweepSphere` every update.
    enum class ProjectileMode : std::uint8_t
    {
        Hitscan,
        Ballistic
    };

    /// Controls what happens after a ballistic impact.
    enum class ProjectileImpactResponse : std::uint8_t
    {
        Destroy,
        Bounce,
        Pierce
    };

    /// Input: gameplay-level projectile parameters in SI-style world units.
    /// Output: a validated description consumed by `ProjectileSystem::Spawn`.
    /// Task: make fast gameplay projectiles explicit rather than silently using
    /// discrete rigid bodies that can tunnel through thin colliders. `Layer`
    /// describes the projectile's gameplay category; `CollisionMask` selects
    /// world categories queried by the projectile. `IgnoredOwnerCollider` is
    /// normally the muzzle/owner collider that must not self-hit.
    struct ProjectileDesc final
    {
        ProjectileMode Mode = ProjectileMode::Ballistic;
        ProjectileImpactResponse Response = ProjectileImpactResponse::Destroy;
        Vec3f Position = Vec3f::Zero();
        Vec3f Velocity = Vec3f{ 20.0f, 0.0f, 0.0f };
        float Radius = 0.05f;
        float Mass = 0.05f;
        float Lifetime = 5.0f;
        float MaxDistance = 100.0f;
        float GravityScale = 1.0f;
        float Restitution = 0.0f;
        std::uint32_t Layer = CollisionLayer::Projectile;
        std::uint32_t CollisionMask = CollisionLayer::All;
        ColliderID IgnoredOwnerCollider = InvalidColliderID;
        bool ApplyImpactImpulse = true;
    };

    /// Immutable result emitted once for every exact projectile impact.
    struct ProjectileHitEvent final
    {
        ProjectileID Projectile = InvalidProjectileID;
        ProjectileMode Mode = ProjectileMode::Ballistic;
        PhysicsSweepHit Sweep;
        Vec3f IncomingVelocity = Vec3f::Zero();
        bool IsTrigger = false;
    };

    /// Queryable state for debug rendering, replay logs, and game code.
    struct ProjectileState final
    {
        ProjectileID ID = InvalidProjectileID;
        bool Active = false;
        ProjectileDesc Desc;
        Vec3f Position = Vec3f::Zero();
        Vec3f Velocity = Vec3f::Zero();
        float Age = 0.0f;
        float DistanceTravelled = 0.0f;
    };

    /// Input: descriptor supplied to `Spawn`.
    /// Output: throws with a concrete field name for invalid physical data.
    /// Task: keep projectile update allocation-free and prevent malformed
    /// gameplay values from entering the continuous-query path.
    inline void ValidateProjectileDesc(const ProjectileDesc& desc)
    {
        RequireFinite(desc.Position, "ProjectileDesc.Position");
        RequireFinite(desc.Velocity, "ProjectileDesc.Velocity");
        RequirePositive(desc.Radius, "ProjectileDesc.Radius");
        RequirePositive(desc.Mass, "ProjectileDesc.Mass");
        RequirePositive(desc.Lifetime, "ProjectileDesc.Lifetime");
        RequirePositive(desc.MaxDistance, "ProjectileDesc.MaxDistance");
        RequireFinite(desc.GravityScale, "ProjectileDesc.GravityScale");
        RequireNonNegative(desc.Restitution, "ProjectileDesc.Restitution");

        if (desc.Restitution > 1.0f)
        {
            throw std::invalid_argument("ProjectileDesc.Restitution must be in [0, 1].");
        }
        if (desc.Layer == 0u || desc.CollisionMask == 0u)
        {
            throw std::invalid_argument("ProjectileDesc.Layer and CollisionMask must each contain a layer bit.");
        }
        if (desc.Velocity.LengthSquared() <= 1.0e-12f)
        {
            throw std::invalid_argument("ProjectileDesc.Velocity must be non-zero.");
        }
    }

    /// Dedicated gameplay projectile subsystem.
    ///
    /// This is intentionally separate from `PhysicsWorld`: a rifle round or
    /// spell bolt is usually a query/lifecycle problem, not a full rigid body.
    /// The system owns stable inactive records, supports callbacks, and routes
    /// collision work through the world’s raycast/swept-sphere API.
    class ProjectileSystem final
    {
    public:
        using HitCallback = std::function<void(const ProjectileHitEvent&)>;

        /// Input: validated projectile description.
        /// Output: stable projectile id; IDs are never compacted or reused.
        /// Task: add a projectile without mutating PhysicsWorld ownership.
        [[nodiscard]]
        ProjectileID Spawn(const ProjectileDesc& desc)
        {
            ValidateProjectileDesc(desc);
            const ProjectileID id = static_cast<ProjectileID>(m_Projectiles.size());
            m_Projectiles.push_back(ProjectileState{
                id, true, desc, desc.Position, desc.Velocity, 0.0f, 0.0f
            });
            return id;
        }

        void Destroy(ProjectileID projectile)
        {
            if (projectile >= m_Projectiles.size())
            {
                throw std::out_of_range("ProjectileSystem::Destroy failed: projectile id does not exist.");
            }
            m_Projectiles.at(projectile).Active = false;
        }

        [[nodiscard]]
        bool IsActive(ProjectileID projectile) const noexcept
        {
            return projectile < m_Projectiles.size() && m_Projectiles[projectile].Active;
        }

        [[nodiscard]]
        const std::vector<ProjectileState>& Projectiles() const noexcept
        {
            return m_Projectiles;
        }

        [[nodiscard]]
        const std::vector<ProjectileHitEvent>& LastHits() const noexcept
        {
            return m_LastHits;
        }

        void SetHitCallback(HitCallback callback)
        {
            m_HitCallback = std::move(callback);
        }

        void ClearHitCallback()
        {
            m_HitCallback = nullptr;
        }

        /// Input: current physics world and positive simulation delta time.
        /// Output: projectiles advance, expire, bounce/pierce/destroy, and emit
        /// deterministic hit events ordered by stable projectile ID.
        /// Task: execute hitscan once and ballistic projectiles continuously.
        /// The world is queried but not stepped here; callers choose whether
        /// projectile motion occurs before or after their rigid-body step.
        void Step(PhysicsWorld& world, float dt)
        {
            RequirePositive(dt, "ProjectileSystem.dt");
            m_LastHits.clear();

            for (ProjectileState& projectile : m_Projectiles)
            {
                if (!projectile.Active)
                {
                    continue;
                }

                if (projectile.Desc.Mode == ProjectileMode::Hitscan)
                {
                    StepHitscan(world, projectile);
                    continue;
                }

                StepBallistic(world, projectile, dt);
            }
        }

    private:
        std::vector<ProjectileState> m_Projectiles;
        std::vector<ProjectileHitEvent> m_LastHits;
        HitCallback m_HitCallback;

        void EmitHit(const ProjectileHitEvent& event)
        {
            m_LastHits.push_back(event);
            if (m_HitCallback)
            {
                m_HitCallback(event);
            }
        }

        void ApplyImpactImpulse(
            PhysicsWorld& world,
            const ProjectileState& projectile,
            const PhysicsSweepHit& hit,
            const Vec3f& incomingVelocity)
        {
            if (!projectile.Desc.ApplyImpactImpulse || hit.IsTrigger || !world.IsValidBody(hit.Body))
            {
                return;
            }

            const RigidBody& target = world.Bodies().at(hit.Body);
            if (!IsDynamic(target))
            {
                return;
            }

            const Vec3f impulse = incomingVelocity * projectile.Desc.Mass;
            world.ApplyBodyImpulseAtPoint(hit.Body, impulse, hit.Point);
        }

        void HandleImpact(
            PhysicsWorld& world,
            ProjectileState& projectile,
            const PhysicsSweepHit& hit,
            const Vec3f& incomingVelocity)
        {
            const ProjectileHitEvent event{
                projectile.ID, projectile.Desc.Mode, hit, incomingVelocity, hit.IsTrigger
            };
            EmitHit(event);
            ApplyImpactImpulse(world, projectile, hit, incomingVelocity);

            if (hit.IsTrigger || projectile.Desc.Response == ProjectileImpactResponse::Pierce)
            {
                const Vec3f direction = SafeNormalize(incomingVelocity, hit.Normal);
                projectile.Position = hit.Point + direction * (projectile.Desc.Radius + 1.0e-3f);
                return;
            }

            if (projectile.Desc.Response == ProjectileImpactResponse::Bounce)
            {
                projectile.Position = hit.Point + hit.Normal * (projectile.Desc.Radius + 1.0e-3f);
                projectile.Velocity = Reflect(incomingVelocity, hit.Normal) * projectile.Desc.Restitution;
                if (projectile.Velocity.LengthSquared() <= 1.0e-10f)
                {
                    projectile.Active = false;
                }
                return;
            }

            projectile.Active = false;
        }

        void StepHitscan(PhysicsWorld& world, ProjectileState& projectile)
        {
            const Vec3f direction = SafeNormalize(projectile.Velocity, Vec3f::UnitX());
            const auto rayHit = world.Raycast(
                projectile.Position,
                direction,
                projectile.Desc.MaxDistance,
                projectile.Desc.CollisionMask,
                projectile.Desc.IgnoredOwnerCollider);

            projectile.DistanceTravelled += rayHit ? rayHit->Distance : projectile.Desc.MaxDistance;
            projectile.Age = projectile.Desc.Lifetime;
            projectile.Active = false;
            if (!rayHit)
            {
                return;
            }

            const PhysicsSweepHit hit{
                rayHit->Body, rayHit->Collider, rayHit->Point, rayHit->Normal,
                rayHit->Distance, rayHit->Distance / projectile.Desc.MaxDistance, rayHit->IsTrigger
            };
            HandleImpact(world, projectile, hit, projectile.Velocity);
        }

        void StepBallistic(PhysicsWorld& world, ProjectileState& projectile, float dt)
        {
            projectile.Velocity += world.Gravity * (projectile.Desc.GravityScale * dt);
            const Vec3f displacement = projectile.Velocity * dt;
            const float distance = displacement.Length();
            if (distance <= 1.0e-10f)
            {
                projectile.Age += dt;
                if (projectile.Age >= projectile.Desc.Lifetime)
                {
                    projectile.Active = false;
                }
                return;
            }

            const auto hit = world.SweepSphere(
                projectile.Position,
                displacement,
                projectile.Desc.Radius,
                projectile.Desc.CollisionMask,
                projectile.Desc.IgnoredOwnerCollider);

            projectile.Age += dt;
            projectile.DistanceTravelled += hit ? hit->Distance : distance;
            if (hit)
            {
                HandleImpact(world, projectile, *hit, projectile.Velocity);
            }
            else
            {
                projectile.Position += displacement;
            }

            if (projectile.DistanceTravelled >= projectile.Desc.MaxDistance ||
                projectile.Age >= projectile.Desc.Lifetime)
            {
                projectile.Active = false;
            }
        }
    };
}
