module;

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
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

    [[nodiscard]]
    inline float BoxExtentAt(
        const Vec3f& halfExtents,
        std::size_t axis) noexcept
    {
        return axis == 0 ? halfExtents.x : (axis == 1 ? halfExtents.y : halfExtents.z);
    }

    [[nodiscard]]
    inline float ProjectBoxRadius(
        const OrientedBoxFrame& box,
        const Vec3f& axis)
    {
        return
            std::abs(Dot(axis, box.Axes[0])) * box.HalfExtents.x +
            std::abs(Dot(axis, box.Axes[1])) * box.HalfExtents.y +
            std::abs(Dot(axis, box.Axes[2])) * box.HalfExtents.z;
    }

    [[nodiscard]]
    inline Vec3f SupportPoint(
        const OrientedBoxFrame& box,
        const Vec3f& direction)
    {
        Vec3f result =
            box.Center;

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            result +=
                box.Axes[axis] *
                (Dot(box.Axes[axis], direction) >= 0.0f
                    ? BoxExtentAt(box.HalfExtents, axis)
                    : -BoxExtentAt(box.HalfExtents, axis));
        }

        return result;
    }

    [[nodiscard]]
    inline Vec3f ClosestPointOnBox(
        const OrientedBoxFrame& box,
        const Vec3f& point)
    {
        const Vec3f delta =
            point - box.Center;

        Vec3f closest =
            box.Center;

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            const float distance =
                std::clamp(
                    Dot(delta, box.Axes[axis]),
                    -BoxExtentAt(box.HalfExtents, axis),
                    BoxExtentAt(box.HalfExtents, axis));

            closest += box.Axes[axis] * distance;
        }

        return closest;
    }

    [[nodiscard]]
    inline Vec3f ClosestPointOnSegment(
        const Vec3f& a,
        const Vec3f& b,
        const Vec3f& point)
    {
        const Vec3f edge = b - a;
        const float lengthSq = edge.LengthSquared();
        if (lengthSq <= 1.0e-12f)
        {
            return a;
        }

        const float t = std::clamp(Dot(point - a, edge) / lengthSq, 0.0f, 1.0f);
        return a + edge * t;
    }

    [[nodiscard]]
    inline std::pair<Vec3f, Vec3f> ClosestPointsOnSegments(
        const Vec3f& a0,
        const Vec3f& a1,
        const Vec3f& b0,
        const Vec3f& b1)
    {
        const Vec3f da = a1 - a0;
        const Vec3f db = b1 - b0;
        const Vec3f offset = a0 - b0;
        const float aa = Dot(da, da);
        const float bb = Dot(db, db);
        const float ab = Dot(da, db);
        const float ao = Dot(da, offset);
        const float bo = Dot(db, offset);
        const float denominator = aa * bb - ab * ab;

        float ta = 0.0f;
        float tb = 0.0f;
        if (aa > 1.0e-12f && bb > 1.0e-12f)
        {
            if (std::abs(denominator) > 1.0e-12f)
            {
                ta = std::clamp((ab * bo - bb * ao) / denominator, 0.0f, 1.0f);
            }
            tb = std::clamp((ab * ta + bo) / bb, 0.0f, 1.0f);
            ta = std::clamp((ab * tb - ao) / aa, 0.0f, 1.0f);
        }
        else if (aa > 1.0e-12f)
        {
            ta = std::clamp(-ao / aa, 0.0f, 1.0f);
        }
        else if (bb > 1.0e-12f)
        {
            tb = std::clamp(bo / bb, 0.0f, 1.0f);
        }

        return { a0 + da * ta, b0 + db * tb };
    }

    [[nodiscard]]
    inline std::pair<Vec3f, Vec3f> ClosestSegmentBoxPoints(
        const CapsuleSegment& capsule,
        const OrientedBoxFrame& box)
    {
        // Squared distance from a point on a segment to a convex box is a
        // convex one-dimensional function. Ternary minimization is stable here
        // and avoids a separate, error-prone segment-vs-OBB clipping path.
        float lower = 0.0f;
        float upper = 1.0f;
        const Vec3f edge = capsule.B - capsule.A;
        for (int iteration = 0; iteration < 32; ++iteration)
        {
            const float first = lower + (upper - lower) / 3.0f;
            const float second = upper - (upper - lower) / 3.0f;
            const Vec3f pointFirst = capsule.A + edge * first;
            const Vec3f pointSecond = capsule.A + edge * second;
            const float distanceFirst = (ClosestPointOnBox(box, pointFirst) - pointFirst).LengthSquared();
            const float distanceSecond = (ClosestPointOnBox(box, pointSecond) - pointSecond).LengthSquared();
            if (distanceFirst <= distanceSecond)
            {
                upper = second;
            }
            else
            {
                lower = first;
            }
        }

        const Vec3f segmentPoint = capsule.A + edge * ((lower + upper) * 0.5f);
        return { segmentPoint, ClosestPointOnBox(box, segmentPoint) };
    }

    [[nodiscard]]
    inline std::optional<ContactPoint> MakeCapsuleSphereContact(
        const CapsuleSegment& capsule,
        const Vec3f& sphereCenter,
        float sphereRadius,
        bool normalFromCapsuleToSphere)
    {
        const Vec3f capsulePoint =
            ClosestPointOnSegment(capsule.A, capsule.B, sphereCenter);
        const Vec3f delta = sphereCenter - capsulePoint;
        const float radiusSum = capsule.Radius + sphereRadius;
        const float distanceSq = delta.LengthSquared();
        if (distanceSq >= radiusSum * radiusSum)
        {
            return std::nullopt;
        }

        const float distance = std::sqrt(std::max(distanceSq, 0.0f));
        const Vec3f capsuleToSphere =
            distance > 1.0e-6f ? delta / distance : Vec3f::Up();
        const Vec3f normal = normalFromCapsuleToSphere ? capsuleToSphere : -capsuleToSphere;
        return MakeContactPoint(
            capsulePoint + capsuleToSphere * (capsule.Radius - 0.5f * (radiusSum - distance)),
            normal,
            radiusSum - distance);
    }

    [[nodiscard]]
    inline std::optional<ContactPoint> MakeCapsuleBoxContact(
        const CapsuleSegment& capsule,
        const OrientedBoxFrame& box,
        bool normalFromCapsuleToBox)
    {
        const auto [capsulePoint, boxPoint] =
            ClosestSegmentBoxPoints(capsule, box);
        const Vec3f delta = boxPoint - capsulePoint;
        const float distanceSq = delta.LengthSquared();
        if (distanceSq >= capsule.Radius * capsule.Radius)
        {
            return std::nullopt;
        }

        const float distance = std::sqrt(std::max(distanceSq, 0.0f));
        const Vec3f capsuleToBox = distance > 1.0e-6f
            ? delta / distance
            : -SafeNormalize(capsulePoint - box.Center, box.Axes[0]);
        const Vec3f normal = normalFromCapsuleToBox ? capsuleToBox : -capsuleToBox;
        return MakeContactPoint(
            (capsulePoint + capsuleToBox * capsule.Radius + boxPoint) * 0.5f,
            normal,
            capsule.Radius - distance);
    }

    [[nodiscard]]
    inline Vec3f SphereBoxFallbackNormal(
        const OrientedBoxFrame& box,
        const Vec3f& sphereCenter)
    {
        const Vec3f delta =
            sphereCenter - box.Center;

        float bestDistance =
            std::numeric_limits<float>::max();

        Vec3f bestNormal =
            box.Axes[0];

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            const float local =
                Dot(delta, box.Axes[axis]);

            const float faceDistance =
                BoxExtentAt(box.HalfExtents, axis) - std::abs(local);

            if (faceDistance < bestDistance)
            {
                bestDistance = faceDistance;
                bestNormal = local >= 0.0f ? box.Axes[axis] : -box.Axes[axis];
            }
        }

        return SafeNormalize(bestNormal, Vec3f::UnitX());
    }

    [[nodiscard]]
    inline std::optional<ContactPoint> MakeSphereBoxContact(
        const Vec3f& sphereCenter,
        float sphereRadius,
        const OrientedBoxFrame& box,
        bool normalFromSphereToBox)
    {
        const Vec3f closest =
            ClosestPointOnBox(box, sphereCenter);

        Vec3f delta =
            closest - sphereCenter;

        float distanceSq =
            delta.LengthSquared();

        Vec3f normalFromSphere =
            Vec3f::UnitX();

        float penetration =
            0.0f;

        if (distanceSq > 1.0e-10f)
        {
            if (distanceSq >= sphereRadius * sphereRadius)
            {
                return std::nullopt;
            }

            const float distance =
                std::sqrt(distanceSq);

            normalFromSphere = delta / distance;
            penetration = sphereRadius - distance;
        }
        else
        {
            normalFromSphere =
                -SphereBoxFallbackNormal(box, sphereCenter);

            penetration = sphereRadius;
        }

        const Vec3f normal =
            normalFromSphereToBox ? normalFromSphere : -normalFromSphere;

        const Vec3f sphereSurface =
            sphereCenter + normalFromSphere * sphereRadius;

        return MakeContactPoint(
            (sphereSurface + closest) * 0.5f,
            normal,
            penetration);
    }

    [[nodiscard]]
    inline bool TestBoxAxis(
        const OrientedBoxFrame& boxA,
        const OrientedBoxFrame& boxB,
        const Vec3f& axis,
        const Vec3f& delta,
        Vec3f& bestNormal,
        float& bestPenetration)
    {
        if (axis.LengthSquared() <= 1.0e-10f)
        {
            return true;
        }

        const Vec3f normal =
            SafeNormalize(axis, Vec3f::UnitX());

        const float distance =
            std::abs(Dot(delta, normal));

        const float overlap =
            ProjectBoxRadius(boxA, normal) +
            ProjectBoxRadius(boxB, normal) -
            distance;

        if (overlap <= 0.0f)
        {
            return false;
        }

        if (overlap < bestPenetration)
        {
            bestPenetration = overlap;
            bestNormal = Dot(delta, normal) >= 0.0f ? normal : -normal;
        }

        return true;
    }

    [[nodiscard]]
    inline bool FindBoxBoxSAT(
        const OrientedBoxFrame& boxA,
        const OrientedBoxFrame& boxB,
        Vec3f& normal,
        float& penetration)
    {
        const Vec3f delta =
            boxB.Center - boxA.Center;

        penetration =
            std::numeric_limits<float>::max();

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            if (!TestBoxAxis(boxA, boxB, boxA.Axes[axis], delta, normal, penetration) ||
                !TestBoxAxis(boxA, boxB, boxB.Axes[axis], delta, normal, penetration))
            {
                return false;
            }
        }

        for (std::size_t a = 0; a < 3; ++a)
        {
            for (std::size_t b = 0; b < 3; ++b)
            {
                if (!TestBoxAxis(
                    boxA,
                    boxB,
                    Cross(boxA.Axes[a], boxB.Axes[b]),
                    delta,
                    normal,
                    penetration))
                {
                    return false;
                }
            }
        }

        return true;
    }

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
                colliderB.ID,
                colliderA.IsTrigger || colliderB.IsTrigger);

        const Vec3f centerA =
            WorldColliderCenter(bodyA, colliderA);

        const Vec3f centerB =
            WorldColliderCenter(bodyB, colliderB);

        if (const auto* capsuleA = std::get_if<CapsuleCollider>(&colliderA.Shape))
        {
            const CapsuleSegment segmentA =
                WorldCapsuleSegment(bodyA, colliderA, *capsuleA);

            if (const auto* sphereB = std::get_if<SphereCollider>(&colliderB.Shape))
            {
                const auto point = MakeCapsuleSphereContact(segmentA, centerB, sphereB->Radius, true);
                if (point) { manifold.Points.push_back(*point); return manifold; }
                return std::nullopt;
            }

            if (const auto* capsuleB = std::get_if<CapsuleCollider>(&colliderB.Shape))
            {
                const CapsuleSegment segmentB = WorldCapsuleSegment(bodyB, colliderB, *capsuleB);
                const auto [pointA, pointB] = ClosestPointsOnSegments(segmentA.A, segmentA.B, segmentB.A, segmentB.B);
                const Vec3f delta = pointB - pointA;
                const float radiusSum = capsuleA->Radius + capsuleB->Radius;
                const float distanceSq = delta.LengthSquared();
                if (distanceSq >= radiusSum * radiusSum) { return std::nullopt; }
                const float distance = std::sqrt(std::max(distanceSq, 0.0f));
                const Vec3f normal = distance > 1.0e-6f ? delta / distance : Vec3f::Up();
                manifold.Points.push_back(MakeContactPoint(
                    pointA + normal * (capsuleA->Radius - 0.5f * (radiusSum - distance)),
                    normal,
                    radiusSum - distance));
                return manifold;
            }

            if (const auto* planeB = std::get_if<PlaneCollider>(&colliderB.Shape))
            {
                const float signedA = Dot(planeB->Normal, segmentA.A) + planeB->Distance;
                const float signedB = Dot(planeB->Normal, segmentA.B) + planeB->Distance;
                const Vec3f closest = signedA <= signedB ? segmentA.A : segmentA.B;
                const float signedDistance = std::min(signedA, signedB);
                const float penetration = capsuleA->Radius - signedDistance;
                if (penetration <= 0.0f) { return std::nullopt; }
                manifold.Points.push_back(MakeContactPoint(
                    closest - planeB->Normal * capsuleA->Radius,
                    -planeB->Normal,
                    penetration));
                return manifold;
            }

            if (const auto* boxB = std::get_if<BoxCollider>(&colliderB.Shape))
            {
                const auto point = MakeCapsuleBoxContact(
                    segmentA, WorldBoxFrame(bodyB, colliderB, boxB->HalfExtents), true);
                if (point) { manifold.Points.push_back(*point); return manifold; }
                return std::nullopt;
            }

            if (const auto* boxB = std::get_if<AABBCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB{ centerB, { Vec3f::UnitX(), Vec3f::UnitY(), Vec3f::UnitZ() }, boxB->HalfExtents };
                const auto point = MakeCapsuleBoxContact(segmentA, frameB, true);
                if (point) { manifold.Points.push_back(*point); return manifold; }
                return std::nullopt;
            }
        }

        if (std::holds_alternative<CapsuleCollider>(colliderB.Shape))
        {
            const auto swapped = CollidePair(bodyB, colliderB, bodyA, colliderA);
            if (!swapped) { return std::nullopt; }
            ContactManifold result = MakeContactManifold(
                bodyA.ID, bodyB.ID, colliderA.ID, colliderB.ID,
                colliderA.IsTrigger || colliderB.IsTrigger);
            for (ContactPoint point : swapped->Points)
            {
                point.Normal = -point.Normal;
                result.Points.push_back(point);
            }
            return result;
        }

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

            if (const auto* boxB = std::get_if<BoxCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB =
                    WorldBoxFrame(bodyB, colliderB, boxB->HalfExtents);

                const auto point =
                    MakeSphereBoxContact(
                        centerA,
                        sphereA->Radius,
                        frameB,
                        true);

                if (!point)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(*point);
                return manifold;
            }

            if (const auto* boxB = std::get_if<AABBCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB
                {
                    centerB,
                    {
                        Vec3f::UnitX(),
                        Vec3f::UnitY(),
                        Vec3f::UnitZ()
                    },
                    boxB->HalfExtents
                };

                const auto point =
                    MakeSphereBoxContact(
                        centerA,
                        sphereA->Radius,
                        frameB,
                        true);

                if (!point)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(*point);
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

            if (std::holds_alternative<BoxCollider>(colliderB.Shape))
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
                        colliderB.ID,
                        colliderA.IsTrigger || colliderB.IsTrigger);

                for (ContactPoint point : swapped->Points)
                {
                    point.Normal = -point.Normal;
                    result.Points.push_back(point);
                }

                return result;
            }
        }

        if (const auto* boxA = std::get_if<BoxCollider>(&colliderA.Shape))
        {
            const OrientedBoxFrame frameA =
                WorldBoxFrame(bodyA, colliderA, boxA->HalfExtents);

            if (const auto* sphereB = std::get_if<SphereCollider>(&colliderB.Shape))
            {
                const auto point =
                    MakeSphereBoxContact(
                        centerB,
                        sphereB->Radius,
                        frameA,
                        false);

                if (!point)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(*point);
                return manifold;
            }

            if (const auto* boxB = std::get_if<BoxCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB =
                    WorldBoxFrame(bodyB, colliderB, boxB->HalfExtents);

                Vec3f normal =
                    Vec3f::UnitX();

                float penetration =
                    0.0f;

                if (!FindBoxBoxSAT(frameA, frameB, normal, penetration))
                {
                    return std::nullopt;
                }

                const Vec3f pointA =
                    SupportPoint(frameA, normal);

                const Vec3f pointB =
                    SupportPoint(frameB, -normal);

                manifold.Points.push_back(
                    MakeContactPoint(
                        (pointA + pointB) * 0.5f,
                        normal,
                        penetration));

                return manifold;
            }

            if (const auto* planeB = std::get_if<PlaneCollider>(&colliderB.Shape))
            {
                const float projectedRadius =
                    ProjectBoxRadius(frameA, planeB->Normal);

                const float signedDistance =
                    Dot(planeB->Normal, frameA.Center) + planeB->Distance;

                const float penetration =
                    projectedRadius - signedDistance;

                if (penetration <= 0.0f)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(
                    MakeContactPoint(
                        SupportPoint(frameA, -planeB->Normal),
                        -planeB->Normal,
                        penetration));

                return manifold;
            }
        }

        if (const auto* boxA = std::get_if<AABBCollider>(&colliderA.Shape))
        {
            if (const auto* sphereB = std::get_if<SphereCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameA
                {
                    centerA,
                    {
                        Vec3f::UnitX(),
                        Vec3f::UnitY(),
                        Vec3f::UnitZ()
                    },
                    boxA->HalfExtents
                };

                const auto point =
                    MakeSphereBoxContact(
                        centerB,
                        sphereB->Radius,
                        frameA,
                        false);

                if (!point)
                {
                    return std::nullopt;
                }

                manifold.Points.push_back(*point);
                return manifold;
            }

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
                    colliderB.ID,
                    colliderA.IsTrigger || colliderB.IsTrigger);

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
