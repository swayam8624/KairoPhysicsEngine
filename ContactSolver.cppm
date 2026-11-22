// TEACHING MAP: PhysicsWorld restores a contact cache and warm-starts
// impulses before SolverIsland invokes SolveContactManifold. This module
// consumes PhysicsMath's ContactNormalDenominator, ApplyImpulseAtPoint and
// ClampAccumulatedImpulseDelta; PhysicsWorld owns response/event policy.
//
// The contact normal points from body A to B and opposite signed impulses
// preserve pair momentum. Nonnegative accumulated normal impulse prevents
// a separating contact from pulling objects together. Tangential response
// is limited by a friction budget, with scalar impulse values cached across
// fixed steps for faster convergence.
//
// This implementation approximates friction: the tangent direction is
// reconstructed from live velocity and reuses the normal effective-mass
// denominator. It is not a complete two-tangent friction model. Triggers
// deliberately produce no solver impulse.
module;

#include <algorithm>
#include <cmath>
#include <cstdint>
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
    // Called inside SolverIsland's velocity-iteration loop. A manifold
    // contains potentially several witness points, each carrying an
    // accumulated normal and tangent impulse. Those scalar accumulators
    // are persisted by PhysicsWorld's contact cache between fixed steps.
    // World policy has already marked Ignore/Trigger/Block, so trigger
    // manifolds are valid events but must produce *no* physical impulse.
    inline void SolveContactManifold(
        std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        ContactManifold& manifold,
        const PhysicsStepSettings& settings,
        float dt)
    {
        if (manifold.IsTrigger)
        {
            return;
        }

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

        // Material parameters belong to individual colliders: two
        // shapes attached to one body can still have different friction
        // or restitution. Resolve them *after* ID and activity checks.
        const PhysicsMaterial materialA =
            colliderA ? colliderA->Material : PhysicsMaterial{};

        const PhysicsMaterial materialB =
            colliderB ? colliderB->Material : PhysicsMaterial{};

        // Material mixing happens after collider-ID lookup; material
        // belongs to a collider rather than implicitly to a parent body.
        const float restitution =
            MixRestitution(materialA, materialB);

        const float dynamicFriction =
            MixFriction(materialA.DynamicFriction, materialB.DynamicFriction);

        for (ContactPoint& point : manifold.Points)
        {
            const Vec3f normal =
                SafeNormalize(point.Normal, Vec3f::Up());

            // Lever arm r = contactWorldPosition - centerOfMassWorld.
            // A force applied at r induces torque r cross impulse,
            // which is why linear inverse mass alone is insufficient.
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

            // Effective mass includes both inverse mass and angular
            // leverage (r x n). World-space inverse inertia means that
            // an off-center contact can change angular velocity.
            // The scalar effective-mass denominator is
            // (mA^-1 + mB^-1) + rotational leverage about the normal.
            // It converts desired relative normal velocity change into
            // an impulse magnitude; near-zero means no solvable motion.
            const float denominator =
                ContactNormalDenominator(
                    bodyA.Mass.InverseMass,
                    inverseInertiaA,
                    rA,
                    bodyB.Mass.InverseMass,
                    inverseInertiaB,
                    rB,
                    normal);

            // Baumgarte turns penetration beyond slop into velocity bias,
            // clamped by settings. It is stabilization, not CCD.
            // Penetration stabilization inserts an extra velocity
            // correction proportional to depth beyond tolerance / dt.
            // Slop avoids jitter for microscopic overlaps; cap keeps
            // a deep collision from requesting an unbounded correction.
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

            // Only approaching/negative biased normal velocity uses the
            // restitution coefficient. Existing separating contacts
            // should not receive an extra bounce from this rule.
            const float restitutionScale =
                biasedNormalVelocity < 0.0f
                    ? (1.0f + restitution)
                    : 1.0f;

            const float requestedNormalImpulse =
                -restitutionScale * biasedNormalVelocity / denominator;

            // Clamp the *accumulated* normal impulse >= 0 and apply only
            // the difference. This supports warm starts and avoids
            // unphysical attraction by unilateral contact constraints.
            // Project old accumulator + requested delta to [0, infinity)
            // and apply the *difference* to each body. This is a
            // unilateral complementarity constraint: contact may push
            // bodies apart but must not glue them together.
            const float normalImpulseDelta =
                ClampAccumulatedImpulseDelta(
                    point.NormalImpulse,
                    requestedNormalImpulse,
                    0.0f,
                    std::numeric_limits<float>::max());

            point.NormalImpulse += normalImpulseDelta;

            // Implementation detail to test: skipping a tiny normal
            // change also skips this point's tangent/friction solve.
            // A more general friction iteration could solve tangential
            // velocity even when the normal accumulator is unchanged.
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

            // Remove the normal component of velocity to obtain the
            // one-dimensional tangent direction used by this solver.
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

                // Approximation: reuse normal effective mass instead of
                // computing tangent effective mass; compare against a
                // two-tangent implementation for oblique/sliding contacts.
                    // This V1 uses a *single live sliding direction* and
                // reuses the normal effective-mass denominator. Exact
                // two-tangent Coulomb friction requires tangential
                // effective masses and a persistent tangent basis.
                const float requestedTangentImpulse =
                    -tangentSpeed / std::max(denominator, 1.0e-6f);

                // Coulomb cone approximated as a scalar interval:
                // |frictionImpulse| <= dynamicFriction * normalImpulse.
                // The normal accumulator must already be nonnegative.
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
    // PhysicsWorld has restored cached scalar impulse values into the
    // manifold points. Reapply these to velocities before iteration,
    // reconstructing the tangent from current relative motion.
    // PhysicsWorld::RestoreContactCache matches previous witnesses
    // using pair IDs and local anchors, then writes these accumulators.
    // Reapply last step's solution before iterative solve to improve
    // rest/stack convergence. The cached tangent is a scalar only;
    // rotating sliding directions across steps is an accuracy limit.
    inline void WarmStartContacts(
        std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        std::vector<ContactManifold>& contacts)
    {
        for (ContactManifold& manifold : contacts)
        {
            if (manifold.IsTrigger)
            {
                continue;
            }

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
    // Standalone all-manifold solver helper. PhysicsWorld's current
    // path partitions into islands and calls SolveContactManifold there.
    // Reference whole-array iteration, useful for isolated solver
    // tests. The main Step path calls SolveSolverIslands to partition
    // bodies and potentially parallelize independent constraint sets.
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
    // Standalone positional helper. PhysicsWorld uses the island-aware
    // correction pass, including joints, rather than invoking this path.
    // Residual penetration fix, distinct from the velocity impulse
    // pass and *not* a continuous collision check. Each iteration
    // distributes a bounded correction by inverse mass; a static
    // body stays fixed while the dynamic body takes the displacement.
    // PhysicsWorld generally invokes the island-aware correction.
    inline void CorrectPositions(
        std::vector<RigidBody>& bodies,
        const std::vector<ContactManifold>& contacts,
        const PhysicsStepSettings& settings)
    {
        for (std::uint32_t iteration = 0; iteration < settings.PositionIterations; ++iteration)
        {
            for (const ContactManifold& manifold : contacts)
            {
                if (manifold.IsTrigger)
                {
                    continue;
                }

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
