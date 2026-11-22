// TEACHING MAP: Convert BroadphasePair (possibly overlapping *bounds*)
// into shape-tested ContactManifold data for PhysicsWorld and ContactSolver.
// Geometry/AABB provides primitive math; Spatial/BVHTraversal narrows triangle
// mesh work; ConvexCollision supplies convex penetration queries.
//
// Coordinate invariants matter: normal direction must agree with manifold
// body A -> B ordering, penetration depth is nonnegative, and collider IDs
// identify the exact shape on possibly multi-collider bodies. Swapping shapes
// therefore requires reversing normals where a reversed query is reused.
//
// Numerical policy is shape-specific: box SAT uses face and cross-edge axes,
// degenerate axes are skipped; closest-point routines clamp finite segments;
// triangle meshes query a BVH and use thin convex prisms for candidates.
// These approximations should not be described as globally exact contact
// geometry. Compare PhysicsEngineTests and PhysicsStressTests before relying
// on edge/deep-overlap behavior or treating timings as correctness evidence.
module;

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Narrowphase;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Geometry.Plane;
import Kairo.Foundation.Spatial.BVHTraversal;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.Broadphase;
import Kairo.Foundation.PhysicsEngine.ConvexCollision;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    [[nodiscard]]
    // Select an OBB's local half-extent by axis index (0/1/2).
    // Caller loops over valid box axes; this convenience helper does not
    // perform bounds checking of arbitrary user-provided axis indices.
    inline float BoxExtentAt(
        const Vec3f& halfExtents,
        std::size_t axis) noexcept
    {
        return axis == 0 ? halfExtents.x : (axis == 1 ? halfExtents.y : halfExtents.z);
    }

    [[nodiscard]]
    // For any (preferably unit) axis, project each OBB half-axis onto it.
    // Sum of absolute projections gives the support radius used by SAT;
    // box center separation then determines overlap on that axis.
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
    // For an oriented box, choose signed half-extent on each world axis
    // based on directional dot. The resulting support is used both by SAT
    // contact fallback and conservative geometric queries.
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
    // Project the world-space point into each normalized box axis, clamp to
    // [-extent,+extent], then reconstruct a world-space closest point.
    // For a point INSIDE the box this returns the point itself; a caller
    // needing a boundary normal must use a separate interior fallback.
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
    // Projection parameter t = dot(p-a,b-a)/|b-a|^2 is clamped to [0,1].
    // The length-squared guard treats a collapsed segment as its endpoint;
    // this is used by capsule/collision helper paths.
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
    // Solve two clamped segment parameters for minimum separation. When
    // nearly parallel or either segment degenerates, use guarded endpoint
    // paths rather than divide by a near-zero denominator. Capsule/capsule
    // contacts depend on these witnesses and shape-order normal convention.
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
    // Minimize squared distance from a capsule's centerline to OBB across
    // segment parameter t in [0,1]. Thirty-two ternary iterations bracket
    // a convex distance function, then report centerline and box witnesses.
    // This is numerical approximation, not exact analytic clipping.
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
    // A capsule is a sphere swept along a finite segment. First project
    // the sphere center onto that segment, then compare center distance
    // with the *sum* of the two radii. This yields the nearest axial
    // contact, not a broadphase bounding-box approximation.
    // normalFromCapsuleToSphere exists because CollidePair dispatches
    // both collider orders but requires a consistent A->B normal.
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
    // ClosestSegmentBoxPoints finds the relevant pair of surface/axis
    // witnesses in the oriented box frame. A capsule contributes its
    // radius after the segment query. The near-zero-distance fallback
    // normal is an explicit degeneracy policy, not a measured direction.
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
    // If a sphere center lies inside an oriented box, ClosestPointOnBox
    // gives zero direction. Select the box face with the least remaining
    // interior margin and use its signed axis as an explicit fallback.
    // This normal choice affects grazing/deep-overlap contact stability.
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
    // Query a closest point on the oriented box, then handle the
    // interior-center special case separately: a zero length delta
    // cannot normalize to a useful contact direction. The bool argument
    // flips A->B convention when shape order is sphere/box vs box/sphere.
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
    // SAT: if either OBB separates along one axis, there is no overlap.
    // The minimum positive overlap gives a candidate response normal.
    // Parallel/near-degenerate cross axes are ignored to avoid unstable
    // normalization; tolerances here affect grazing-contact behavior.
    // Separating-axis theorem: project both oriented half-box extents
    // onto a candidate axis and compare their interval overlap.
    // Any negative overlap *separates* the boxes, so return false.
    // Positive overlap may improve the minimum-penetration normal.
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
    // For OBBs in 3D, test three face axes from each box plus
    // nine pairwise edge-cross axes: up to 15 candidate separating axes.
    // Even a positive SAT result still needs representative contact points.
    // For two general oriented boxes, test 3 face normals from A,
    // 3 from B and 9 pairwise edge-cross axes (at most 15 axes).
    // Near-parallel cross products are skipped in TestBoxAxis.
    // SAT proves overlap for ideal boxes; generating a stable limited
    // contact manifold is separate work in BuildBoxBoxContacts.
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

    [[nodiscard]]
    inline std::array<Vec3f, 8> BoxVertices(
        const OrientedBoxFrame& box)
    {
        std::array<Vec3f, 8> vertices{};
        std::size_t index = 0u;
        for (int x = -1; x <= 1; x += 2)
        {
            for (int y = -1; y <= 1; y += 2)
            {
                for (int z = -1; z <= 1; z += 2)
                {
                    vertices[index++] =
                        box.Center +
                        box.Axes[0] * (box.HalfExtents.x * static_cast<float>(x)) +
                        box.Axes[1] * (box.HalfExtents.y * static_cast<float>(y)) +
                        box.Axes[2] * (box.HalfExtents.z * static_cast<float>(z));
                }
            }
        }
        return vertices;
    }

    [[nodiscard]]
    inline bool PointInsideBox(
        const OrientedBoxFrame& box,
        const Vec3f& point,
        float tolerance = 1.0e-4f) noexcept
    {
        const Vec3f delta = point - box.Center;
        for (std::size_t axis = 0u; axis < 3u; ++axis)
        {
            if (std::abs(Dot(delta, box.Axes[axis])) >
                BoxExtentAt(box.HalfExtents, axis) + tolerance)
            {
                return false;
            }
        }
        return true;
    }

    // Merge witnesses that are almost colocated and nearly collinear
    // in normal, retaining the deepest. This removes duplicate contacts
    // from adjacent box vertices/mesh triangles before iterative solve.
    // Thresholds are heuristic numerical tolerances in world units.
    inline void AppendUniqueContact(
        std::vector<ContactPoint>& points,
        const ContactPoint& candidate)
    {
        constexpr float mergeDistanceSq = 1.0e-8f;
        for (ContactPoint& existing : points)
        {
            if ((existing.Position - candidate.Position).LengthSquared() <= mergeDistanceSq &&
                Dot(SafeNormalize(existing.Normal, Vec3f::Up()),
                    SafeNormalize(candidate.Normal, Vec3f::Up())) > 0.99f)
            {
                if (candidate.PenetrationDepth > existing.PenetrationDepth)
                {
                    existing = candidate;
                }
                return;
            }
        }
        points.push_back(candidate);
    }

    // Keep the deepest contact first, then repeatedly prefer points
    // farthest from existing choices. This spreads a small manifold over
    // the patch for solver stability; it is not exact polygon clipping.
    // The solver's contact budget is four points. Preserve the deepest
    // first, then spread remaining witnesses over the patch by a
    // farthest-point heuristic. The result is deterministic under the
    // same candidate iteration order, but not exact polygon clipping.
    inline void ReduceContactPoints(
        std::vector<ContactPoint>& points,
        std::size_t maxPoints = 4u)
    {
        if (points.size() <= maxPoints)
        {
            return;
        }

        std::vector<ContactPoint> reduced;
        reduced.reserve(maxPoints);
        std::vector<bool> selected(points.size(), false);

        std::size_t deepest = 0u;
        for (std::size_t i = 1u; i < points.size(); ++i)
        {
            if (points[i].PenetrationDepth > points[deepest].PenetrationDepth)
            {
                deepest = i;
            }
        }
        reduced.push_back(points[deepest]);
        selected[deepest] = true;

        while (reduced.size() < maxPoints)
        {
            std::size_t best = points.size();
            float bestSpread = -1.0f;
            for (std::size_t i = 0u; i < points.size(); ++i)
            {
                if (selected[i])
                {
                    continue;
                }

                float nearestSq = std::numeric_limits<float>::max();
                for (const ContactPoint& chosen : reduced)
                {
                    nearestSq = std::min(
                        nearestSq,
                        (points[i].Position - chosen.Position).LengthSquared());
                }

                if (nearestSq > bestSpread)
                {
                    bestSpread = nearestSq;
                    best = i;
                }
            }

            if (best == points.size())
            {
                break;
            }
            selected[best] = true;
            reduced.push_back(points[best]);
        }

        points = std::move(reduced);
    }

    // Given SAT's normal/penetration, collect vertices from A inside B
    // and from B inside A, estimate a representative surface position,
    // deduplicate, and reduce to a solver-friendly manifold.
    // An edge-edge overlap may require a fallback contact even when
    // neither box contributes an interior vertex.
    inline void BuildBoxBoxContacts(
        ContactManifold& manifold,
        const OrientedBoxFrame& boxA,
        const OrientedBoxFrame& boxB,
        const Vec3f& collisionNormal,
        float penetration)
    {
        const Vec3f normal =
            SafeNormalize(collisionNormal, Vec3f::UnitX());
        const float halfPenetration = penetration * 0.5f;

        for (const Vec3f& vertex : BoxVertices(boxA))
        {
            if (PointInsideBox(boxB, vertex))
            {
                AppendUniqueContact(
                    manifold.Points,
                    MakeContactPoint(
                        vertex - normal * halfPenetration,
                        normal,
                        penetration));
            }
        }

        for (const Vec3f& vertex : BoxVertices(boxB))
        {
            if (PointInsideBox(boxA, vertex))
            {
                AppendUniqueContact(
                    manifold.Points,
                    MakeContactPoint(
                        vertex + normal * halfPenetration,
                        normal,
                        penetration));
            }
        }

        if (manifold.Points.empty())
        {
            const Vec3f pointA = SupportPoint(boxA, normal);
            const Vec3f pointB = SupportPoint(boxB, -normal);
            manifold.Points.push_back(
                MakeContactPoint(
                    (pointA + pointB) * 0.5f,
                    normal,
                    penetration));
        }

        ReduceContactPoints(manifold.Points, 4u);
    }

    // A plane is unbounded and therefore has no broadphase tree AABB.
    // For every penetrating box vertex, use signed plane distance to
    // create a witness. Collider-order dispatch must orient the normal
    // consistently so impulse signs in ContactSolver remain correct.
    inline void BuildBoxPlaneContacts(
        ContactManifold& manifold,
        const OrientedBoxFrame& box,
        const PlaneCollider& plane)
    {
        const Vec3f planeNormal =
            SafeNormalize(plane.Normal, Vec3f::Up());

        for (const Vec3f& vertex : BoxVertices(box))
        {
            const float signedDistance =
                Dot(planeNormal, vertex) + plane.Distance;
            if (signedDistance < 0.0f)
            {
                AppendUniqueContact(
                    manifold.Points,
                    MakeContactPoint(
                        vertex - planeNormal * (signedDistance * 0.5f),
                        -planeNormal,
                        -signedDistance));
            }
        }

        if (manifold.Points.empty())
        {
            const float projectedRadius = ProjectBoxRadius(box, planeNormal);
            const float signedCenter = Dot(planeNormal, box.Center) + plane.Distance;
            const float penetration = projectedRadius - signedCenter;
            if (penetration > 0.0f)
            {
                manifold.Points.push_back(
                    MakeContactPoint(
                        SupportPoint(box, -planeNormal),
                        -planeNormal,
                        penetration));
            }
        }

        ReduceContactPoints(manifold.Points, 4u);
    }

    [[nodiscard]]
    // Convert a world AABB to a conservative AABB in the mesh collider's
    // local frame: transform all eight corners by inverse collider pose,
    // then recompute min/max. Used to query a local-space triangle BVH;
    // never pass a world box directly as local triangle indices.
    inline AABBf WorldBoundsToColliderLocal(
        const RigidBody& body,
        const Collider& collider,
        const AABBf& worldBounds)
    {
        const Vec3f center = WorldColliderCenter(body, collider);
        const Quaternionf inverse = WorldColliderRotation(body, collider).Conjugate();
        AABBf local = AABBf::Empty();
        for (std::uint32_t mask = 0u; mask < 8u; ++mask)
        {
  const Vec3f corner
  {
      (mask & 1u) != 0u ? worldBounds.Max.x : worldBounds.Min.x,
      (mask & 2u) != 0u ? worldBounds.Max.y : worldBounds.Min.y,
      (mask & 4u) != 0u ? worldBounds.Max.z : worldBounds.Min.z
  };
  local.ExpandToInclude(Rotate(inverse, corner - center));
        }
        return local;
    }

    [[nodiscard]]
    // Give a zero-thickness mesh triangle a small authored surface volume
    // by extruding ±SurfaceThickness/2 along its normal. Construct a
    // six-vertex closed triangular prism so CollideConvex can support-map it.
    // Mesh-vs-convex physics here is approximate, not exact infinitesimal
    // triangle collision or soft-body self-intersection.
    inline ConvexHullCollider MakeTriangleCollisionPrism(
        const TriangleMeshCollider& mesh,
        std::size_t triangleIndex)
    {
        const auto& triangle = mesh.Triangles.at(triangleIndex);
        const Vec3f a = mesh.Vertices.at(triangle[0]);
        const Vec3f b = mesh.Vertices.at(triangle[1]);
        const Vec3f c = mesh.Vertices.at(triangle[2]);
        const Vec3f normal = SafeNormalize(Cross(b - a, c - a), Vec3f::Up());
        const Vec3f offset = normal * (mesh.SurfaceThickness * 0.5f);
        return {
  { a + offset, b + offset, c + offset,
    a - offset, b - offset, c - offset },
  {
      { 0u, 1u, 2u }, { 3u, 5u, 4u },
      { 0u, 3u, 4u }, { 0u, 4u, 1u },
      { 1u, 4u, 5u }, { 1u, 5u, 2u },
      { 2u, 5u, 3u }, { 2u, 3u, 0u }
  }
        };
    }

    [[nodiscard]]
    // A triangulated mesh may query finite support-mapped peer shapes, but
    // planes and another triangle mesh use separate dispatch paths.
    // This is a type policy only; activity, dimensions and correctness of
    // collision geometry are validated by other functions.
    inline bool IsTriangleMeshConvexPeer(const Collider& collider) noexcept
    {
        return !std::holds_alternative<PlaneCollider>(collider.Shape) &&
  !std::holds_alternative<TriangleMeshCollider>(collider.Shape);
    }

    [[nodiscard]]
    // Broad triangle mesh/mesh path: BVH AABB queries supply triangle IDs;
    // individual triangle prisms are checked with convex penetration and
    // only the deepest candidate is retained. This is an approximation,
    // not an exhaustive triangle/triangle contact manifold.
    // Both mesh acceleration structures are queried conservatively;
    // candidate triangles are converted to thin convex prisms for
    // penetration tests. This selects a deepest representative hit,
    // not an exhaustive manifold over all intersecting mesh triangles.
    // Never interpret this approximate mesh path as arbitrary deformable
    // self-collision or a watertight continuous collision oracle.
    inline std::optional<ContactManifold> CollideTriangleMeshes(
        const RigidBody& bodyA,
        const Collider& colliderA,
        const TriangleMeshCollider& meshA,
        const RigidBody& bodyB,
        const Collider& colliderB,
        const TriangleMeshCollider& meshB)
    {
        if (meshA.Acceleration.Empty() || meshB.Acceleration.Empty())
            return std::nullopt;

        const AABBf boundsB = WorldAABB(bodyB, colliderB);
        if (!boundsB.IsValid()) return std::nullopt;

        const AABBf queryA =
            WorldBoundsToColliderLocal(bodyA, colliderA, boundsB);
        const auto candidatesA = kairo::foundation::spatial::QueryAABB(
            meshA.Acceleration, queryA);

        std::optional<ConvexPenetration> deepest;
        for (const auto triangleA : candidatesA.PrimitiveIndices)
        {
            if (triangleA >= meshA.Triangles.size()) continue;

            Collider prismA = colliderA;
            prismA.Shape = MakeTriangleCollisionPrism(meshA, triangleA);
            const AABBf prismABounds = WorldAABB(bodyA, prismA);
            if (!prismABounds.IsValid()) continue;

            const AABBf queryB =
                WorldBoundsToColliderLocal(bodyB, colliderB, prismABounds);
            const auto candidatesB = kairo::foundation::spatial::QueryAABB(
                meshB.Acceleration, queryB);

            for (const auto triangleB : candidatesB.PrimitiveIndices)
            {
                if (triangleB >= meshB.Triangles.size()) continue;

                Collider prismB = colliderB;
                prismB.Shape = MakeTriangleCollisionPrism(meshB, triangleB);
                const auto penetration =
                    CollideConvex(bodyA, prismA, bodyB, prismB);
                if (penetration &&
                    (!deepest || penetration->Depth > deepest->Depth))
                {
                    deepest = penetration;
                }
            }
        }

        if (!deepest) return std::nullopt;

        ContactManifold manifold = MakeContactManifold(
            bodyA.ID, bodyB.ID, colliderA.ID, colliderB.ID,
            colliderA.IsTrigger || colliderB.IsTrigger);
        manifold.Points.push_back(MakeContactPoint(
            deepest->Position, deepest->Normal, deepest->Depth));
        return manifold;
    }

    [[nodiscard]]
    // Mesh/convex path: transform peer bounds into mesh-local space for
    // BVH traversal; collision prisms approximate candidate triangles.
    // Check the peer-shape policy below before adding new collider types.
    // Transform the convex peer's world AABB into the mesh collider's
    // *local* frame before BVH traversal. Triangle indices refer to
    // stored local mesh primitives; the convex-penetration primitive
    // adapter converts candidates into consistent pair-space witnesses.
    inline std::optional<ContactManifold> CollideTriangleMesh(
        const RigidBody& meshBody,
        const Collider& meshCollider,
        const TriangleMeshCollider& mesh,
        const RigidBody& otherBody,
        const Collider& otherCollider)
    {
        if (!IsTriangleMeshConvexPeer(otherCollider)) return std::nullopt;

        const AABBf otherBounds = WorldAABB(otherBody, otherCollider);
        if (!otherBounds.IsValid()) return std::nullopt;
        const AABBf localQuery =
  WorldBoundsToColliderLocal(meshBody, meshCollider, otherBounds);
        const auto candidates = kairo::foundation::spatial::QueryAABB(
  mesh.Acceleration, localQuery);

        std::optional<ConvexPenetration> deepest;
        for (const auto triangleIndex : candidates.PrimitiveIndices)
        {
  if (triangleIndex >= mesh.Triangles.size()) continue;
  Collider prism = meshCollider;
  prism.Shape = MakeTriangleCollisionPrism(mesh, triangleIndex);
  const auto penetration =
      CollideConvex(meshBody, prism, otherBody, otherCollider);
  if (penetration && (!deepest || penetration->Depth > deepest->Depth))
      deepest = penetration;
        }

        if (!deepest) return std::nullopt;
        ContactManifold manifold = MakeContactManifold(
  meshBody.ID, otherBody.ID, meshCollider.ID, otherCollider.ID,
  meshCollider.IsTrigger || otherCollider.IsTrigger);
        manifold.Points.push_back(MakeContactPoint(
  deepest->Position, deepest->Normal, deepest->Depth));
        return manifold;
    }

    /// Input: two colliders and their owning bodies.
    /// Output: contact manifold when the shapes overlap.
    /// Task: dispatch shape-specific contact tests after broadphase candidate
    /// pairing; mesh prism and reduced-manifold paths are approximations.
    [[nodiscard]]
    // Central shape-dispatch boundary. A positive broadphase pair
    // enters here and may still produce no manifold. Cases include
    // sphere, box, capsule, plane, convex and supported mesh peers;
    // reversed shape order must reverse contact normals to preserve
    // the solver's bodyA -> bodyB sign convention.
    // The trigger flag records *non-impulsive* contact intent; world
    // pair/layer/callback response policy may override it later.
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

        if (const auto* meshA = std::get_if<TriangleMeshCollider>(&colliderA.Shape))
        {
            if (const auto* meshB = std::get_if<TriangleMeshCollider>(&colliderB.Shape))
            {
                return CollideTriangleMeshes(
                    bodyA, colliderA, *meshA, bodyB, colliderB, *meshB);
            }

            return CollideTriangleMesh(
                bodyA, colliderA, *meshA, bodyB, colliderB);
        }

        if (std::holds_alternative<TriangleMeshCollider>(colliderB.Shape))
        {
            const auto swapped = CollidePair(bodyB, colliderB, bodyA, colliderA);
            if (!swapped) return std::nullopt;
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

        if (std::holds_alternative<ConvexHullCollider>(colliderA.Shape))
        {
            if (const auto* planeB = std::get_if<PlaneCollider>(&colliderB.Shape))
            {
                const Vec3f hullPoint =
                    ConvexShapeSupport(bodyA, colliderA, -planeB->Normal);
                const float signedDistance =
                    Dot(planeB->Normal, hullPoint) + planeB->Distance;
                if (signedDistance >= 0.0f) return std::nullopt;
                manifold.Points.push_back(MakeContactPoint(
                    hullPoint, -planeB->Normal, -signedDistance));
                return manifold;
            }

            const auto penetration =
                CollideConvex(bodyA, colliderA, bodyB, colliderB);
            if (!penetration) return std::nullopt;
            manifold.Points.push_back(MakeContactPoint(
                penetration->Position,
                penetration->Normal,
                penetration->Depth));
            return manifold;
        }

        if (std::holds_alternative<ConvexHullCollider>(colliderB.Shape))
        {
            const auto swapped = CollidePair(bodyB, colliderB, bodyA, colliderA);
            if (!swapped) return std::nullopt;
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

                BuildBoxBoxContacts(
                    manifold,
                    frameA,
                    frameB,
                    normal,
                    penetration);
                return manifold;
            }


            if (const auto* boxB = std::get_if<AABBCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB
                {
                    centerB,
                    { Vec3f::UnitX(), Vec3f::UnitY(), Vec3f::UnitZ() },
                    boxB->HalfExtents
                };
                Vec3f normal = Vec3f::UnitX();
                float penetration = 0.0f;
                if (!FindBoxBoxSAT(frameA, frameB, normal, penetration))
                {
                    return std::nullopt;
                }
                BuildBoxBoxContacts(manifold, frameA, frameB, normal, penetration);
                return manifold;
            }

            if (const auto* planeB = std::get_if<PlaneCollider>(&colliderB.Shape))
            {
                BuildBoxPlaneContacts(manifold, frameA, *planeB);
                if (manifold.Points.empty())
                {
                    return std::nullopt;
                }
                return manifold;
            }
        }

        if (const auto* boxA = std::get_if<AABBCollider>(&colliderA.Shape))
        {
            const OrientedBoxFrame axisAlignedFrameA
            {
                centerA,
                { Vec3f::UnitX(), Vec3f::UnitY(), Vec3f::UnitZ() },
                boxA->HalfExtents
            };
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

            if (const auto* boxB = std::get_if<BoxCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB =
                    WorldBoxFrame(bodyB, colliderB, boxB->HalfExtents);
                Vec3f normal = Vec3f::UnitX();
                float penetration = 0.0f;
                if (!FindBoxBoxSAT(axisAlignedFrameA, frameB, normal, penetration))
                {
                    return std::nullopt;
                }
                BuildBoxBoxContacts(manifold, axisAlignedFrameA, frameB, normal, penetration);
                return manifold;
            }

            if (const auto* boxB = std::get_if<AABBCollider>(&colliderB.Shape))
            {
                const OrientedBoxFrame frameB
                {
                    centerB,
                    { Vec3f::UnitX(), Vec3f::UnitY(), Vec3f::UnitZ() },
                    boxB->HalfExtents
                };
                Vec3f normal = Vec3f::UnitX();
                float penetration = 0.0f;
                if (!FindBoxBoxSAT(axisAlignedFrameA, frameB, normal, penetration))
                {
                    return std::nullopt;
                }
                BuildBoxBoxContacts(manifold, axisAlignedFrameA, frameB, normal, penetration);
                return manifold;
            }

            if (const auto* planeB = std::get_if<PlaneCollider>(&colliderB.Shape))
            {
                BuildBoxPlaneContacts(manifold, axisAlignedFrameA, *planeB);
                if (manifold.Points.empty())
                {
                    return std::nullopt;
                }
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
    // Pair indices were produced by BroadphaseWorld and are assumed valid
    // for the supplied collider vector (the .at calls will throw otherwise).
    // Recheck liveness and body ownership since a candidate isn't a contact.
    // Empty optional/point sets are not forwarded to the iterative solver.
    // Convert the deduplicated broadphase pair list into real,
    // shape-tested manifolds. Recheck active collider/body ownership,
    // because tree candidates are caches and can contain noncontacts.
    // PhysicsWorld then resolves Ignore/Trigger/Block policy *before*
    // warm starting, solving, or publishing contact events.
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
