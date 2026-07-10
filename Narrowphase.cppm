module;

#include <algorithm>
#include <cmath>
#include <optional>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Narrowphase;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.Plane;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.Broadphase;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    /// Input: two colliders and their owning bodies.
    /// Output: contact manifold when the shapes overlap.
    /// Task: perform exact V1 collision tests after broadphase candidate pairing.
    [[nodiscard]]
    inline std::optional<ContactManifold> CollidePair(
        const RigidBody& bodyA,
        const Collider& colliderA,
        const RigidBody& bodyB,
        const Collider& colliderB)
    {
        if (!IsActiveBody(bodyA) ||
            !IsActiveBody(bodyB) ||
            !IsActiveCollider(colliderA) ||
            !IsActiveCollider(colliderB))
        {
            return std::nullopt;
        }

        ContactManifold manifold =
            MakeContactManifold(
                bodyA.ID,
                bodyB.ID,
                colliderA.ID,
                colliderB.ID);

        const Vec3f centerA =
            WorldColliderCenter(bodyA, colliderA);

        const Vec3f centerB =
            WorldColliderCenter(bodyB, colliderB);

        if (const auto* sphereA = std::get_if<SphereCollider>(&colliderA.Shape))
        {
            if (const auto* sphereB = std::get_if<SphereCollider>(&colliderB.Shape))
            {
                const Vec3f delta =
                    centerB - centerA;

                const float radiusSum =
                    sphereA->Radius + sphereB->Radius;

                const float distanceSq =
                    delta.LengthSquared();

                if (distanceSq >= radiusSum * radiusSum)
                {
                    return std::nullopt;
                }

                const float distance =
                    std::sqrt(std::max(distanceSq, 0.0f));

                const Vec3f normal =
                    distance > 1.0e-6f ? delta / distance : Vec3f::Up();

                manifold.Points.push_back(
                    MakeContactPoint(
                        centerA + normal * (sphereA->Radius - 0.5f * (radiusSum - distance)),
                        normal,
                        radiusSum - distance));

                return manifold;
            }

            if (const auto* planeB = std::get_if<PlaneCollider>(&colliderB.Shape))
            {
                const float signedDistance =
                    Dot(planeB->Normal, centerA) + planeB->Distance;

                const float penetration =
                    sphereA->Radius - signedDistance;

                if (penetration <= 0.0f)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(
                    MakeContactPoint(
                        centerA - planeB->Normal * sphereA->Radius,
                        -planeB->Normal,
                        penetration));

                return manifold;
            }
        }

        if (const auto* planeA = std::get_if<PlaneCollider>(&colliderA.Shape))
        {
            if (const auto* sphereB = std::get_if<SphereCollider>(&colliderB.Shape))
            {
                const float signedDistance =
                    Dot(planeA->Normal, centerB) + planeA->Distance;

                const float penetration =
                    sphereB->Radius - signedDistance;

                if (penetration <= 0.0f)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(
                    MakeContactPoint(
                        centerB - planeA->Normal * sphereB->Radius,
                        planeA->Normal,
                        penetration));

                return manifold;
            }
        }

        if (const auto* boxA = std::get_if<AABBCollider>(&colliderA.Shape))
        {
            if (const auto* boxB = std::get_if<AABBCollider>(&colliderB.Shape))
            {
                const Vec3f delta =
                    centerB - centerA;

                const Vec3f overlap
                {
                    boxA->HalfExtents.x + boxB->HalfExtents.x - std::abs(delta.x),
                    boxA->HalfExtents.y + boxB->HalfExtents.y - std::abs(delta.y),
                    boxA->HalfExtents.z + boxB->HalfExtents.z - std::abs(delta.z)
                };

                if (overlap.x <= 0.0f || overlap.y <= 0.0f || overlap.z <= 0.0f)
                {
                    return std::nullopt;
                }

                Vec3f normal = delta.x >= 0.0f ? Vec3f::UnitX() : -Vec3f::UnitX();
                float penetration = overlap.x;
                if (overlap.y < penetration)
                {
                    normal = delta.y >= 0.0f ? Vec3f::UnitY() : -Vec3f::UnitY();
                    penetration = overlap.y;
                }
                if (overlap.z < penetration)
                {
                    normal = delta.z >= 0.0f ? Vec3f::UnitZ() : -Vec3f::UnitZ();
                    penetration = overlap.z;
                }

                manifold.Points.push_back(
                    MakeContactPoint(
                        (centerA + centerB) * 0.5f,
                        normal,
                        penetration));

                return manifold;
            }

            if (const auto* planeB = std::get_if<PlaneCollider>(&colliderB.Shape))
            {
                const float projectedRadius =
                    std::abs(planeB->Normal.x) * boxA->HalfExtents.x +
                    std::abs(planeB->Normal.y) * boxA->HalfExtents.y +
                    std::abs(planeB->Normal.z) * boxA->HalfExtents.z;

                const float signedDistance =
                    Dot(planeB->Normal, centerA) + planeB->Distance;

                const float penetration =
                    projectedRadius - signedDistance;

                if (penetration <= 0.0f)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(
                    MakeContactPoint(
                        centerA - planeB->Normal * projectedRadius,
                        -planeB->Normal,
                        penetration));

                return manifold;
            }
        }

        if (std::holds_alternative<PlaneCollider>(colliderA.Shape) &&
            std::holds_alternative<AABBCollider>(colliderB.Shape))
        {
            const auto swapped =
                CollidePair(bodyB, colliderB, bodyA, colliderA);

            if (!swapped)
            {
                return std::nullopt;
            }

            ContactManifold result =
                MakeContactManifold(
                    bodyA.ID,
                    bodyB.ID,
                    colliderA.ID,
                    colliderB.ID);

            for (ContactPoint point : swapped->Points)
            {
                point.Normal = -point.Normal;
                result.Points.push_back(point);
            }

            return result;
        }

        return std::nullopt;
    }

    /// Input: broadphase pairs plus body/collider arrays.
    /// Output: all exact contact manifolds.
    /// Task: convert candidate pairs into solver-ready contact data.
    [[nodiscard]]
    inline std::vector<ContactManifold> ComputeContacts(
        const std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        const std::vector<BroadphasePair>& pairs)
    {
        std::vector<ContactManifold> contacts;

        for (const BroadphasePair& pair : pairs)
        {
            const Collider& a =
                colliders.at(pair.A);

            const Collider& b =
                colliders.at(pair.B);

            if (!IsActiveCollider(a) ||
                !IsActiveCollider(b) ||
                a.Body >= bodies.size() ||
                b.Body >= bodies.size() ||
                !IsActiveBody(bodies.at(a.Body)) ||
                !IsActiveBody(bodies.at(b.Body)) ||
                a.Body == b.Body)
            {
                continue;
            }

            const auto contact =
                CollidePair(
                    bodies.at(a.Body),
                    a,
                    bodies.at(b.Body),
                    b);

            if (contact && !contact->Points.empty())
            {
                contacts.push_back(*contact);
            }
        }

        return contacts;
    }
}
