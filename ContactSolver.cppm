module;

#include <algorithm>
#include <cmath>
#include <limits>
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

    /// Input: mutable bodies, colliders, one mutable manifold, and timestep.
    /// Output: body velocities and contact accumulated impulses updated.
    /// Task: solve one contact manifold with sequential impulses. The contact
    /// points are mutable because commercial solvers cache accumulated normal
    /// and tangent impulse values for warm starting on the next fixed step.
    inline void SolveContactManifold(
        std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        ContactManifold& manifold,
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

        if (!IsActiveBody(bodyA) || !IsActiveBody(bodyB))
        {
            return;
        }

        if (!IsDynamic(bodyA) && !IsDynamic(bodyB))
        {
            return;
        }

        const Collider* colliderA =
            manifold.ColliderA < colliders.size() &&
                IsActiveCollider(colliders.at(manifold.ColliderA))
                ? &colliders.at(manifold.ColliderA)
                : nullptr;

        const Collider* colliderB =
            manifold.ColliderB < colliders.size() &&
                IsActiveCollider(colliders.at(manifold.ColliderB))
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

        for (ContactPoint& point : manifold.Points)
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

            if (denominator <= 1.0e-6f)
            {
                continue;
            }

            const float biasedNormalVelocity =
                normalVelocity + bias;

            const float restitutionScale =
                biasedNormalVelocity < 0.0f
                    ? (1.0f + restitution)
                    : 1.0f;

            const float requestedNormalImpulse =
                -restitutionScale * biasedNormalVelocity / denominator;

            const float normalImpulseDelta =
                ClampAccumulatedImpulseDelta(
                    point.NormalImpulse,
                    requestedNormalImpulse,
                    0.0f,
                    std::numeric_limits<float>::max());

            point.NormalImpulse += normalImpulseDelta;

            if (std::abs(normalImpulseDelta) <= 1.0e-8f)
            {
                continue;
            }

            const Vec3f impulse =
                normal * normalImpulseDelta;

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

            const Vec3f tangentVelocity =
                postRelativeVelocity - normal * Dot(postRelativeVelocity, normal);

            const float tangentSpeed =
                tangentVelocity.Length();

            Vec3f frictionImpulse =
                Vec3f::Zero();

            if (tangentSpeed > 1.0e-6f)
            {
                const Vec3f tangent =
                    tangentVelocity / tangentSpeed;

                const float requestedTangentImpulse =
                    -tangentSpeed / std::max(denominator, 1.0e-6f);

                const float maxTangentImpulse =
                    dynamicFriction * point.NormalImpulse;

                const float tangentImpulseDelta =
                    ClampAccumulatedImpulseDelta(
                        point.TangentImpulse,
                        requestedTangentImpulse,
                        -maxTangentImpulse,
                        maxTangentImpulse);

                point.TangentImpulse += tangentImpulseDelta;
                frictionImpulse = tangent * tangentImpulseDelta;
            }

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

    /// Input: bodies, colliders, and contacts whose points already contain
    /// cached impulse magnitudes.
    /// Output: body velocities pre-conditioned by the cached impulses.
    /// Task: warm start the solver so resting contacts and stacks converge in
    /// fewer iterations. Tangent direction is reconstructed from the current
    /// relative velocity, which keeps the cache compact and deterministic.
    inline void WarmStartContacts(
        std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        std::vector<ContactManifold>& contacts)
    {
        for (ContactManifold& manifold : contacts)
        {
            if (manifold.BodyA >= bodies.size() || manifold.BodyB >= bodies.size())
            {
                continue;
            }

            if (manifold.ColliderA >= colliders.size() || manifold.ColliderB >= colliders.size())
            {
                continue;
            }

            RigidBody& bodyA =
                bodies.at(manifold.BodyA);

            RigidBody& bodyB =
                bodies.at(manifold.BodyB);

            if (!IsActiveBody(bodyA) ||
                !IsActiveBody(bodyB) ||
                !IsActiveCollider(colliders.at(manifold.ColliderA)) ||
                !IsActiveCollider(colliders.at(manifold.ColliderB)))
            {
                continue;
            }

            const Matrix3f inverseInertiaA =
                WorldInverseInertia(bodyA);

            const Matrix3f inverseInertiaB =
                WorldInverseInertia(bodyB);

            for (ContactPoint& point : manifold.Points)
            {
                if (point.NormalImpulse <= 0.0f &&
                    std::abs(point.TangentImpulse) <= 1.0e-8f)
                {
                    continue;
                }

                const Vec3f normal =
                    SafeNormalize(point.Normal, Vec3f::Up());

                Vec3f tangentImpulse =
                    Vec3f::Zero();

                const Vec3f relativeVelocity =
                    VelocityAtPoint(bodyB.State, point.Position) -
                    VelocityAtPoint(bodyA.State, point.Position);

                const Vec3f tangentVelocity =
                    relativeVelocity - normal * Dot(relativeVelocity, normal);

                const float tangentSpeed =
                    tangentVelocity.Length();

                if (tangentSpeed > 1.0e-6f)
                {
                    tangentImpulse =
                        (tangentVelocity / tangentSpeed) * point.TangentImpulse;
                }

                const Vec3f totalImpulse =
                    normal * point.NormalImpulse + tangentImpulse;

                if (IsDynamic(bodyA))
                {
                    ApplyImpulseAtPoint(
                        bodyA.State,
                        bodyA.Mass.InverseMass,
                        inverseInertiaA,
                        -totalImpulse,
                        point.Position);
                }

                if (IsDynamic(bodyB))
                {
                    ApplyImpulseAtPoint(
                        bodyB.State,
                        bodyB.Mass.InverseMass,
                        inverseInertiaB,
                        totalImpulse,
                        point.Position);
                }
            }
        }
    }

    /// Input: bodies, colliders, contacts, settings, and dt.
    /// Output: body velocities modified over multiple solver iterations.
    /// Task: provide a V1 sequential impulse solver without owning the world.
    inline void SolveContacts(
        std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        std::vector<ContactManifold>& contacts,
        const PhysicsStepSettings& settings,
        float dt)
    {
        for (std::uint32_t iteration = 0; iteration < settings.VelocityIterations; ++iteration)
        {
            for (ContactManifold& manifold : contacts)
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
        for (std::uint32_t iteration = 0; iteration < settings.PositionIterations; ++iteration)
        {
            for (const ContactManifold& manifold : contacts)
            {
                if (manifold.BodyA >= bodies.size() || manifold.BodyB >= bodies.size())
                {
                    continue;
                }

                RigidBody& bodyA = bodies.at(manifold.BodyA);
                RigidBody& bodyB = bodies.at(manifold.BodyB);
                if (!IsActiveBody(bodyA) || !IsActiveBody(bodyB))
                {
                    continue;
                }

                const float inverseMassSum =
                    bodyA.Mass.InverseMass + bodyB.Mass.InverseMass;

                if (inverseMassSum <= 0.0f)
                {
                    continue;
                }

                for (const ContactPoint& point : manifold.Points)
                {
                    const float totalCorrectionMagnitude =
                        std::min(
                            std::max(point.PenetrationDepth - settings.Slop, 0.0f) / inverseMassSum,
                            settings.MaxPositionCorrection);

                    const float correctionMagnitude =
                        totalCorrectionMagnitude /
                        static_cast<float>(settings.PositionIterations);

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
}
