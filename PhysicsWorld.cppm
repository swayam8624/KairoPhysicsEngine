module;

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
            if (body >= m_Bodies.size())
            {
                throw std::out_of_range("AddCollider failed: body id does not exist.");
            }

            const ColliderID id =
                static_cast<ColliderID>(m_Colliders.size());

            m_Colliders.push_back(
                MakeCollider(id, body, shape, material, localCenter));

            m_Broadphase.AddOrUpdateCollider(m_Bodies, m_Colliders.back());
            return id;
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
    };
}
