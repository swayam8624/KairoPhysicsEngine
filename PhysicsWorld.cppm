module;

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.World;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Types;
import Kairo.Foundation.PhysicsEngine.Material;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.Broadphase;
import Kairo.Foundation.PhysicsEngine.Narrowphase;
import Kairo.Foundation.PhysicsEngine.ContactSolver;
import Kairo.Foundation.PhysicsEngine.Debug;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    class PhysicsWorld final
    {
    public:
        PhysicsStepSettings Settings;
        Vec3f Gravity = DefaultGravity;

        [[nodiscard]]
        bool IsValidBody(
            BodyID body) const noexcept
        {
            return body < m_Bodies.size() &&
                IsActiveBody(m_Bodies[body]);
        }

        [[nodiscard]]
        bool IsValidCollider(
            ColliderID collider) const noexcept
        {
            return collider < m_Colliders.size() &&
                IsActiveCollider(m_Colliders[collider]) &&
                IsValidBody(m_Colliders[collider].Body);
        }

        [[nodiscard]]
        BodyID CreateRigidBody(
            const RigidBodyDesc& desc)
        {
            const BodyID id =
                static_cast<BodyID>(m_Bodies.size());

            m_Bodies.push_back(MakeRigidBody(id, desc));
            return id;
        }

        [[nodiscard]]
        ColliderID AddCollider(
            BodyID body,
            ColliderShape shape,
            PhysicsMaterial material = {},
            const Vec3f& localCenter = Vec3f::Zero())
        {
            if (!IsValidBody(body))
            {
                throw std::out_of_range("AddCollider failed: body id does not exist or is inactive.");
            }

            const ColliderID id =
                static_cast<ColliderID>(m_Colliders.size());

            m_Colliders.push_back(
                MakeCollider(id, body, shape, material, localCenter));

            m_Broadphase.AddOrUpdateCollider(m_Bodies, m_Colliders.back());
            return id;
        }

        /// Input: active collider id.
        /// Output: none.
        /// Task: remove the collider from simulation while preserving stable
        /// vector-index ids for debug logs, cached contacts, and user handles.
        void RemoveCollider(
            ColliderID collider)
        {
            if (collider >= m_Colliders.size())
            {
                throw std::out_of_range("RemoveCollider failed: collider id does not exist.");
            }

            Collider& record =
                m_Colliders.at(collider);

            if (!record.Active)
            {
                return;
            }

            record.Active = false;
            record.Body = InvalidBodyID;
            m_Broadphase.RemoveCollider(collider);
            RemoveCachedPairsForCollider(collider);
            RemoveCachedContactsForCollider(collider);
        }

        /// Input: collider id plus category and collision masks.
        /// Output: collider filter updated and broadphase proxy refreshed.
        /// Task: implement commercial-style collision filtering where
        /// `BelongsTo` describes what the collider is and `CollidesWith`
        /// describes what categories it accepts.
        void SetCollisionFilter(
            ColliderID collider,
            std::uint32_t belongsTo,
            std::uint32_t collidesWith)
        {
            if (!IsValidCollider(collider))
            {
                throw std::out_of_range("SetCollisionFilter failed: collider id does not exist or is inactive.");
            }

            Collider& record =
                m_Colliders.at(collider);

            record.BelongsTo = belongsTo;
            record.CollidesWith = collidesWith;
            record.LayerMask = belongsTo;
            m_Broadphase.AddOrUpdateCollider(m_Bodies, record);
        }

        /// Input: collider id and trigger state.
        /// Output: collider marked as solid or sensor.
        /// Task: triggers still produce contact manifolds for gameplay/events,
        /// but the solver skips their impulses and position correction.
        void SetColliderTrigger(
            ColliderID collider,
            bool isTrigger)
        {
            if (!IsValidCollider(collider))
            {
                throw std::out_of_range("SetColliderTrigger failed: collider id does not exist or is inactive.");
            }

            m_Colliders.at(collider).IsTrigger = isTrigger;
        }

        /// Input: active body id.
        /// Output: none.
        /// Task: deactivate a body and every collider attached to it without
        /// compacting storage. This gives users deletion-safe ids: an old id can
        /// be tested with `IsValidBody` and never aliases a future body.
        void DestroyRigidBody(
            BodyID body)
        {
            if (body >= m_Bodies.size())
            {
                throw std::out_of_range("DestroyRigidBody failed: body id does not exist.");
            }

            RigidBody& record =
                m_Bodies.at(body);

            if (!record.Active)
            {
                return;
            }

            for (Collider& collider : m_Colliders)
            {
                if (collider.Active && collider.Body == body)
                {
                    RemoveCollider(collider.ID);
                }
            }

            record.Active = false;
            record.Sleeping = true;
            record.State.LinearVelocity = Vec3f::Zero();
            record.State.AngularVelocity = Vec3f::Zero();
            ClearForces(record.Forces);
            RemoveCachedContactsForBody(body);
        }

        /// Input: active body id.
        /// Output: body marked awake for the next integration pass.
        /// Task: expose an explicit wake hook for gameplay code, tools, and
        /// tests without requiring direct mutation of the body record.
        void WakeBody(
            BodyID body)
        {
            if (!IsValidBody(body))
            {
                throw std::out_of_range("WakeBody failed: body id does not exist or is inactive.");
            }

            WakeRigidBody(m_Bodies.at(body));
        }

        void Step(
            float dt)
        {
            ValidateStepSettings(Settings, dt);
            RequireFinite(Gravity, "Gravity");

            IntegrateForces(dt);
            IntegrateKinematicVelocities(dt);

            m_Broadphase.Sync(m_Bodies, m_Colliders);
            m_LastPairs =
                m_Broadphase.ComputePairs(m_Bodies, m_Colliders);

            m_LastContacts =
                ComputeContacts(m_Bodies, m_Colliders, m_LastPairs);

            WakeContactBodies();
            RestoreContactCache();
            WarmStartContacts(m_Bodies, m_Colliders, m_LastContacts);

            SolveContacts(
                m_Bodies,
                m_Colliders,
                m_LastContacts,
                Settings,
                dt);

            StoreContactCache();

            CorrectPositions(
                m_Bodies,
                m_LastContacts,
                Settings);

            IntegrateDynamicVelocities(dt);
            UpdateSleeping(dt);
            ClearForceAccumulators();
        }

        /// Input: variable elapsed time, fixed simulation dt, and substep cap.
        /// Output: number of fixed `Step` calls executed.
        /// Task: let applications feed variable frame times while the physics
        /// simulation advances in deterministic fixed-size increments.
        std::uint32_t StepFixed(
            float elapsedTime,
            float fixedDt,
            std::uint32_t maxSubSteps = 8)
        {
            RequireNonNegative(elapsedTime, "elapsedTime");
            RequirePositive(fixedDt, "fixedDt");

            if (maxSubSteps == 0)
            {
                throw std::invalid_argument("maxSubSteps must be greater than zero.");
            }

            m_FixedAccumulator += elapsedTime;

            std::uint32_t steps = 0;
            while (m_FixedAccumulator >= fixedDt && steps < maxSubSteps)
            {
                Step(fixedDt);
                m_FixedAccumulator -= fixedDt;
                ++steps;
            }

            if (steps == maxSubSteps && m_FixedAccumulator >= fixedDt)
            {
                m_FixedAccumulator = 0.0f;
            }

            return steps;
        }

        [[nodiscard]]
        float FixedAccumulator() const noexcept
        {
            return m_FixedAccumulator;
        }

        void ResetFixedAccumulator() noexcept
        {
            m_FixedAccumulator = 0.0f;
        }

        [[nodiscard]]
        const std::vector<RigidBody>& Bodies() const noexcept
        {
            return m_Bodies;
        }

        [[nodiscard]]
        std::vector<RigidBody>& Bodies() noexcept
        {
            return m_Bodies;
        }

        [[nodiscard]]
        const std::vector<Collider>& Colliders() const noexcept
        {
            return m_Colliders;
        }

        [[nodiscard]]
        const std::vector<ContactManifold>& Contacts() const noexcept
        {
            return m_LastContacts;
        }

        [[nodiscard]]
        const std::vector<BroadphasePair>& BroadphasePairs() const noexcept
        {
            return m_LastPairs;
        }

        [[nodiscard]]
        std::vector<DebugAABB> DebugAABBs() const
        {
            return CollectDebugAABBs(m_Bodies, m_Colliders);
        }

        [[nodiscard]]
        std::vector<DebugContact> DebugContacts() const
        {
            return CollectDebugContacts(m_LastContacts);
        }

    private:
        std::vector<RigidBody> m_Bodies;
        std::vector<Collider> m_Colliders;
        BroadphaseWorld m_Broadphase;
        std::vector<BroadphasePair> m_LastPairs;
        std::vector<ContactManifold> m_LastContacts;
        float m_FixedAccumulator = 0.0f;

        struct ContactCacheEntry final
        {
            BodyID BodyA = InvalidBodyID;
            BodyID BodyB = InvalidBodyID;
            ColliderID ColliderA = InvalidColliderID;
            ColliderID ColliderB = InvalidColliderID;
            std::uint32_t PointIndex = 0;
            float NormalImpulse = 0.0f;
            float TangentImpulse = 0.0f;
        };

        std::vector<ContactCacheEntry> m_ContactCache;

        void IntegrateForces(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (body.Sleeping && HasAccumulatedForce(body))
                {
                    WakeRigidBody(body);
                }

                if (!IsDynamic(body))
                {
                    continue;
                }

                const Vec3f acceleration =
                    (body.EnableGravity ? Gravity * (Settings.GravityScale * body.GravityScale) : Vec3f::Zero()) +
                    body.Forces.Force * body.Mass.InverseMass;

                body.State.LinearVelocity += acceleration * dt;
                body.State.AngularVelocity +=
                    WorldInverseInertia(body) * body.Forces.Torque * dt;

                ApplyVelocityControls(body, dt);
            }
        }

        void IntegrateKinematicVelocities(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (!IsKinematic(body))
                {
                    continue;
                }

                ApplyVelocityControls(body, dt);
                AdvanceMotionState(body.State, dt);
            }
        }

        void IntegrateDynamicVelocities(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (!IsDynamic(body))
                {
                    continue;
                }

                ApplyVelocityControls(body, dt);
                AdvanceMotionState(body.State, dt);
            }
        }

        void ClearForceAccumulators() noexcept
        {
            for (RigidBody& body : m_Bodies)
            {
                ClearForces(body.Forces);
            }
        }

        static void AdvanceMotionState(
            MotionState& state,
            float dt)
        {
            state.Position += state.LinearVelocity * dt;

            if (state.AngularVelocity.LengthSquared() > 1.0e-12f)
            {
                state.Rotation =
                    IntegrateAngularVelocity(
                        state.Rotation,
                        state.AngularVelocity,
                        dt);
            }
        }

        static void ClampVelocity(
            Vec3f& velocity,
            float maxSpeed)
        {
            const float maxSpeedSq =
                maxSpeed * maxSpeed;

            const float lengthSq =
                velocity.LengthSquared();

            if (lengthSq > maxSpeedSq)
            {
                velocity =
                    velocity * (maxSpeed / std::sqrt(lengthSq));
            }
        }

        static void ApplyVelocityControls(
            RigidBody& body,
            float dt)
        {
            if (body.LinearDamping > 0.0f)
            {
                body.State.LinearVelocity *=
                    std::exp(-body.LinearDamping * dt);
            }

            if (body.AngularDamping > 0.0f)
            {
                body.State.AngularVelocity *=
                    std::exp(-body.AngularDamping * dt);
            }

            ClampVelocity(body.State.LinearVelocity, body.MaxLinearSpeed);
            ClampVelocity(body.State.AngularVelocity, body.MaxAngularSpeed);
        }

        void WakeContactBodies()
        {
            for (const ContactManifold& manifold : m_LastContacts)
            {
                if (manifold.BodyA >= m_Bodies.size() ||
                    manifold.BodyB >= m_Bodies.size())
                {
                    continue;
                }

                RigidBody& bodyA =
                    m_Bodies.at(manifold.BodyA);

                RigidBody& bodyB =
                    m_Bodies.at(manifold.BodyB);

                const bool wakeA =
                    IsDynamicBodyType(bodyA) &&
                    bodyA.Sleeping &&
                    IsDynamicBodyType(bodyB) &&
                    !bodyB.Sleeping;

                const bool wakeB =
                    IsDynamicBodyType(bodyB) &&
                    bodyB.Sleeping &&
                    IsDynamicBodyType(bodyA) &&
                    !bodyA.Sleeping;

                if (wakeA)
                {
                    WakeRigidBody(bodyA);
                }

                if (wakeB)
                {
                    WakeRigidBody(bodyB);
                }
            }
        }

        void UpdateSleeping(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (!IsDynamicBodyType(body))
                {
                    continue;
                }

                if (!Settings.EnableSleeping || !body.AllowSleeping)
                {
                    WakeRigidBody(body);
                    continue;
                }

                if (HasAccumulatedForce(body))
                {
                    WakeRigidBody(body);
                    continue;
                }

                const bool slowLinear =
                    body.State.LinearVelocity.LengthSquared() <=
                    Settings.SleepLinearSpeed * Settings.SleepLinearSpeed;

                const bool slowAngular =
                    body.State.AngularVelocity.LengthSquared() <=
                    Settings.SleepAngularSpeed * Settings.SleepAngularSpeed;

                if (slowLinear && slowAngular)
                {
                    body.SleepTimer += dt;
                    if (body.SleepTimer >= Settings.SleepTime)
                    {
                        body.Sleeping = true;
                        body.State.LinearVelocity = Vec3f::Zero();
                        body.State.AngularVelocity = Vec3f::Zero();
                    }
                }
                else
                {
                    WakeRigidBody(body);
                }
            }
        }

        void RemoveCachedPairsForCollider(
            ColliderID collider)
        {
            m_LastPairs.erase(
                std::remove_if(
                    m_LastPairs.begin(),
                    m_LastPairs.end(),
                    [collider](const BroadphasePair& pair)
                    {
                        return pair.A == collider || pair.B == collider;
                    }),
                m_LastPairs.end());
        }

        void RemoveCachedContactsForCollider(
            ColliderID collider)
        {
            m_LastContacts.erase(
                std::remove_if(
                    m_LastContacts.begin(),
                    m_LastContacts.end(),
                    [collider](const ContactManifold& manifold)
                    {
                        return manifold.ColliderA == collider ||
                            manifold.ColliderB == collider;
                    }),
                m_LastContacts.end());

            m_ContactCache.erase(
                std::remove_if(
                    m_ContactCache.begin(),
                    m_ContactCache.end(),
                    [collider](const ContactCacheEntry& entry)
                    {
                        return entry.ColliderA == collider ||
                            entry.ColliderB == collider;
                    }),
                m_ContactCache.end());
        }

        void RemoveCachedContactsForBody(
            BodyID body)
        {
            m_LastContacts.erase(
                std::remove_if(
                    m_LastContacts.begin(),
                    m_LastContacts.end(),
                    [body](const ContactManifold& manifold)
                    {
                        return manifold.BodyA == body ||
                            manifold.BodyB == body;
                    }),
                m_LastContacts.end());

            m_ContactCache.erase(
                std::remove_if(
                    m_ContactCache.begin(),
                    m_ContactCache.end(),
                    [body](const ContactCacheEntry& entry)
                    {
                        return entry.BodyA == body ||
                            entry.BodyB == body;
                    }),
                m_ContactCache.end());
        }

        [[nodiscard]]
        static bool SameCacheKey(
            const ContactCacheEntry& entry,
            const ContactManifold& manifold,
            std::uint32_t pointIndex) noexcept
        {
            return entry.BodyA == manifold.BodyA &&
                entry.BodyB == manifold.BodyB &&
                entry.ColliderA == manifold.ColliderA &&
                entry.ColliderB == manifold.ColliderB &&
                entry.PointIndex == pointIndex;
        }

        void RestoreContactCache()
        {
            for (ContactManifold& manifold : m_LastContacts)
            {
                for (std::size_t pointIndex = 0; pointIndex < manifold.Points.size(); ++pointIndex)
                {
                    ContactPoint& point =
                        manifold.Points.at(pointIndex);

                    const std::uint32_t stablePointIndex =
                        static_cast<std::uint32_t>(pointIndex);

                    const auto found =
                        std::find_if(
                            m_ContactCache.begin(),
                            m_ContactCache.end(),
                            [&manifold, stablePointIndex](const ContactCacheEntry& entry)
                            {
                                return SameCacheKey(entry, manifold, stablePointIndex);
                            });

                    if (found != m_ContactCache.end())
                    {
                        point.NormalImpulse = found->NormalImpulse;
                        point.TangentImpulse = found->TangentImpulse;
                    }
                }
            }
        }

        void StoreContactCache()
        {
            m_ContactCache.clear();

            for (const ContactManifold& manifold : m_LastContacts)
            {
                if (manifold.IsTrigger)
                {
                    continue;
                }

                for (std::size_t pointIndex = 0; pointIndex < manifold.Points.size(); ++pointIndex)
                {
                    const ContactPoint& point =
                        manifold.Points.at(pointIndex);

                    if (point.NormalImpulse <= 0.0f &&
                        std::abs(point.TangentImpulse) <= 1.0e-8f)
                    {
                        continue;
                    }

                    m_ContactCache.push_back(
                        ContactCacheEntry
                        {
                            manifold.BodyA,
                            manifold.BodyB,
                            manifold.ColliderA,
                            manifold.ColliderB,
                            static_cast<std::uint32_t>(pointIndex),
                            point.NormalImpulse,
                            point.TangentImpulse
                        });
                }
            }

            std::sort(
                m_ContactCache.begin(),
                m_ContactCache.end(),
                [](const ContactCacheEntry& lhs, const ContactCacheEntry& rhs)
                {
                    if (lhs.BodyA != rhs.BodyA)
                    {
                        return lhs.BodyA < rhs.BodyA;
                    }
                    if (lhs.BodyB != rhs.BodyB)
                    {
                        return lhs.BodyB < rhs.BodyB;
                    }
                    if (lhs.ColliderA != rhs.ColliderA)
                    {
                        return lhs.ColliderA < rhs.ColliderA;
                    }
                    if (lhs.ColliderB != rhs.ColliderB)
                    {
                        return lhs.ColliderB < rhs.ColliderB;
                    }
                    return lhs.PointIndex < rhs.PointIndex;
                });
        }
    };
}
