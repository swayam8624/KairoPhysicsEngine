module;

#include <algorithm>
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

        void Step(
            float dt)
        {
            ValidateStepSettings(Settings, dt);
            RequireFinite(Gravity, "Gravity");

            IntegrateForces(dt);

            m_Broadphase.Sync(m_Bodies, m_Colliders);
            m_LastPairs =
                m_Broadphase.ComputePairs(m_Bodies, m_Colliders);

            m_LastContacts =
                ComputeContacts(m_Bodies, m_Colliders, m_LastPairs);

            SolveContacts(
                m_Bodies,
                m_Colliders,
                m_LastContacts,
                Settings,
                dt);

            CorrectPositions(
                m_Bodies,
                m_LastContacts,
                Settings);

            IntegrateVelocities(dt);
            ClearForceAccumulators();
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

        void IntegrateForces(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (!IsDynamic(body))
                {
                    continue;
                }

                const Vec3f acceleration =
                    Gravity * Settings.GravityScale +
                    body.Forces.Force * body.Mass.InverseMass;

                body.State.LinearVelocity += acceleration * dt;
                body.State.AngularVelocity +=
                    WorldInverseInertia(body) * body.Forces.Torque * dt;
            }
        }

        void IntegrateVelocities(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (!IsDynamic(body))
                {
                    continue;
                }

                body.State.Position += body.State.LinearVelocity * dt;

                if (body.State.AngularVelocity.LengthSquared() > 1.0e-12f)
                {
                    body.State.Rotation =
                        IntegrateAngularVelocity(
                            body.State.Rotation,
                            body.State.AngularVelocity,
                            dt);
                }
            }
        }

        void ClearForceAccumulators() noexcept
        {
            for (RigidBody& body : m_Bodies)
            {
                ClearForces(body.Forces);
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
        }
    };
}
