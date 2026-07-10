module;

#include <algorithm>
#include <cmath>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.ContactSolver;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Types;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.Material;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    /// Input: body array, collider array, contact manifold, and dt.
    /// Output: body velocities modified by normal and friction impulses.
    /// Task: solve one contact manifold with a sequential impulse update.
    inline void SolveContactManifold(
        std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        const ContactManifold& manifold,
        const PhysicsStepSettings& settings,
        float dt)
    {
        if (manifold.BodyA >= bodies.size() || manifold.BodyB >= bodies.size())
        {
            return;
        }

        RigidBody& bodyA =
            bodies.at(manifold.BodyA);

        RigidBody& bodyB =
            bodies.at(manifold.BodyB);

        if (!IsDynamic(bodyA) && !IsDynamic(bodyB))
        {
            return;
        }

        const Collider* colliderA =
            manifold.ColliderA < colliders.size()
                ? &colliders.at(manifold.ColliderA)
                : nullptr;

        const Collider* colliderB =
            manifold.ColliderB < colliders.size()
                ? &colliders.at(manifold.ColliderB)
                : nullptr;

        const PhysicsMaterial materialA =
            colliderA ? colliderA->Material : PhysicsMaterial{};

        const PhysicsMaterial materialB =
            colliderB ? colliderB->Material : PhysicsMaterial{};

        const float restitution =
            MixRestitution(materialA, materialB);

        const float dynamicFriction =
            MixFriction(materialA.DynamicFriction, materialB.DynamicFriction);

        for (const ContactPoint& point : manifold.Points)
        {
            const Vec3f normal =
                SafeNormalize(point.Normal, Vec3f::Up());

            const Vec3f rA =
                point.Position - bodyA.State.Position;

            const Vec3f rB =
                point.Position - bodyB.State.Position;

            const Vec3f velocityA =
                VelocityAtPoint(bodyA.State, point.Position);

            const Vec3f velocityB =
                VelocityAtPoint(bodyB.State, point.Position);

            const Vec3f relativeVelocity =
                velocityB - velocityA;

            const Matrix3f inverseInertiaA =
                WorldInverseInertia(bodyA);

            const Matrix3f inverseInertiaB =
                WorldInverseInertia(bodyB);

            const float denominator =
                ContactNormalDenominator(
                    bodyA.Mass.InverseMass,
                    inverseInertiaA,
                    rA,
                    bodyB.Mass.InverseMass,
                    inverseInertiaB,
                    rB,
                    normal);

            const float bias =
                BaumgarteBias(
                    std::max(point.PenetrationDepth - settings.Slop, 0.0f),
                    settings.Baumgarte,
                    dt,
                    settings.MaxPositionCorrection / dt);

            const float normalVelocity =
                Dot(relativeVelocity, normal);

            float normalImpulseMagnitude =
                ComputeNormalImpulseMagnitude(
                    normalVelocity + bias,
                    restitution,
                    denominator);

            if (normalImpulseMagnitude <= 0.0f)
            {
                continue;
            }

            const Vec3f impulse =
                normal * normalImpulseMagnitude;

            if (IsDynamic(bodyA))
            {
                ApplyImpulseAtPoint(
                    bodyA.State,
                    bodyA.Mass.InverseMass,
                    inverseInertiaA,
                    -impulse,
                    point.Position);
            }

            if (IsDynamic(bodyB))
            {
                ApplyImpulseAtPoint(
                    bodyB.State,
                    bodyB.Mass.InverseMass,
                    inverseInertiaB,
                    impulse,
                    point.Position);
            }

            const Vec3f postRelativeVelocity =
                VelocityAtPoint(bodyB.State, point.Position) -
                VelocityAtPoint(bodyA.State, point.Position);

            const Vec3f frictionImpulse =
                ComputeFrictionImpulse(
                    postRelativeVelocity,
                    normal,
                    std::max(denominator, 1.0e-6f),
                    dynamicFriction * normalImpulseMagnitude);

            if (IsDynamic(bodyA))
            {
                ApplyImpulseAtPoint(
                    bodyA.State,
                    bodyA.Mass.InverseMass,
                    inverseInertiaA,
                    -frictionImpulse,
                    point.Position);
            }

            if (IsDynamic(bodyB))
            {
                ApplyImpulseAtPoint(
                    bodyB.State,
                    bodyB.Mass.InverseMass,
                    inverseInertiaB,
                    frictionImpulse,
                    point.Position);
            }
        }
    }

    /// Input: bodies, colliders, contacts, settings, and dt.
    /// Output: body velocities modified over multiple solver iterations.
    /// Task: provide a V1 sequential impulse solver without owning the world.
    inline void SolveContacts(
        std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        const std::vector<ContactManifold>& contacts,
        const PhysicsStepSettings& settings,
        float dt)
    {
        for (std::uint32_t iteration = 0; iteration < settings.VelocityIterations; ++iteration)
        {
            for (const ContactManifold& manifold : contacts)
            {
                SolveContactManifold(bodies, colliders, manifold, settings, dt);
            }
        }
    }

    /// Input: bodies and contacts.
    /// Output: body positions nudged apart according to inverse mass.
    /// Task: correct residual overlap after velocity solving without adding
    /// another engine system.
    inline void CorrectPositions(
        std::vector<RigidBody>& bodies,
        const std::vector<ContactManifold>& contacts,
        const PhysicsStepSettings& settings)
    {
        for (const ContactManifold& manifold : contacts)
        {
            if (manifold.BodyA >= bodies.size() || manifold.BodyB >= bodies.size())
            {
                continue;
            }

            RigidBody& bodyA = bodies.at(manifold.BodyA);
            RigidBody& bodyB = bodies.at(manifold.BodyB);
            const float inverseMassSum =
                bodyA.Mass.InverseMass + bodyB.Mass.InverseMass;

            if (inverseMassSum <= 0.0f)
            {
                continue;
            }

            for (const ContactPoint& point : manifold.Points)
            {
                const float correctionMagnitude =
                    std::min(
                        std::max(point.PenetrationDepth - settings.Slop, 0.0f) / inverseMassSum,
                        settings.MaxPositionCorrection);

                const Vec3f correction =
                    SafeNormalize(point.Normal, Vec3f::Up()) * correctionMagnitude;

                if (IsDynamic(bodyA))
                {
                    bodyA.State.Position -= correction * bodyA.Mass.InverseMass;
                }

                if (IsDynamic(bodyB))
                {
                    bodyB.State.Position += correction * bodyB.Mass.InverseMass;
                }
            }
        }
    }
}
