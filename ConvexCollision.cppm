module;

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.ConvexCollision;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    /// One point on the Minkowski difference A - B, retaining the original
    /// witnesses needed to reconstruct a world-space contact after EPA.
    struct ConvexSupportVertex final
    {
        Vec3f Difference = Vec3f::Zero();
        Vec3f PointA = Vec3f::Zero();
        Vec3f PointB = Vec3f::Zero();
    };

    struct ConvexSimplex final
    {
        std::array<ConvexSupportVertex, 4> Points{};
        std::size_t Count = 0u;

        void PushFront(const ConvexSupportVertex& point)
        {
            for (std::size_t index = std::min(Count, Points.size() - 1u);
                index > 0u; --index)
                Points[index] = Points[index - 1u];
            Points[0] = point;
            Count = std::min(Count + 1u, Points.size());
        }
    };

    struct ConvexPenetration final
    {
        Vec3f Position = Vec3f::Zero();
        Vec3f Normal = Vec3f::UnitX();
        float Depth = 0.0f;
        std::uint32_t GJKIterations = 0u;
        std::uint32_t EPAIterations = 0u;
    };

    [[nodiscard]] inline Vec3f ConvexShapeSupport(
        const RigidBody& body,
        const Collider& collider,
        const Vec3f& direction)
    {
        const Vec3f fallback = Vec3f::UnitX();
        const Vec3f unit = SafeNormalize(direction, fallback);
        const Vec3f center = WorldColliderCenter(body, collider);

        if (const auto* sphere = std::get_if<SphereCollider>(&collider.Shape))
            return center + unit * sphere->Radius;

        if (const auto* capsule = std::get_if<CapsuleCollider>(&collider.Shape))
        {
            const CapsuleSegment segment =
                WorldCapsuleSegment(body, collider, *capsule);
            return (Dot(segment.B - segment.A, direction) >= 0.0f
                ? segment.B : segment.A) + unit * capsule->Radius;
        }

        if (const auto* box = std::get_if<AABBCollider>(&collider.Shape))
            return center + Vec3f{
                direction.x >= 0.0f ? box->HalfExtents.x : -box->HalfExtents.x,
                direction.y >= 0.0f ? box->HalfExtents.y : -box->HalfExtents.y,
                direction.z >= 0.0f ? box->HalfExtents.z : -box->HalfExtents.z };

        if (const auto* box = std::get_if<BoxCollider>(&collider.Shape))
        {
            const OrientedBoxFrame frame =
                WorldBoxFrame(body, collider, box->HalfExtents);
            Vec3f result = frame.Center;
            for (std::size_t axis = 0u; axis < 3u; ++axis)
            {
                const float extent = axis == 0u ? frame.HalfExtents.x
                    : (axis == 1u ? frame.HalfExtents.y : frame.HalfExtents.z);
                result += frame.Axes[axis] *
                    (Dot(frame.Axes[axis], direction) >= 0.0f ? extent : -extent);
            }
            return result;
        }

        if (const auto* hull = std::get_if<ConvexHullCollider>(&collider.Shape))
        {
            const Quaternionf rotation = WorldColliderRotation(body, collider);
            Vec3f best = center + Rotate(rotation, hull->Vertices.front());
            float bestProjection = Dot(best, direction);
            for (std::size_t index = 1u; index < hull->Vertices.size(); ++index)
            {
                const Vec3f candidate =
                    center + Rotate(rotation, hull->Vertices[index]);
                const float projection = Dot(candidate, direction);
                if (projection > bestProjection)
                {
                    best = candidate;
                    bestProjection = projection;
                }
            }
            return best;
        }

        throw std::invalid_argument(
            "Infinite planes do not provide finite convex support points.");
    }

    [[nodiscard]] inline ConvexSupportVertex ConvexMinkowskiSupport(
        const RigidBody& bodyA,
        const Collider& colliderA,
        const RigidBody& bodyB,
        const Collider& colliderB,
        const Vec3f& direction)
    {
        const Vec3f pointA = ConvexShapeSupport(bodyA, colliderA, direction);
        const Vec3f pointB = ConvexShapeSupport(bodyB, colliderB, -direction);
        return { pointA - pointB, pointA, pointB };
    }

    [[nodiscard]] inline Vec3f ClosestPointOnConvexTriangle(
        const Vec3f& point,
        const Vec3f& a,
        const Vec3f& b,
        const Vec3f& c)
    {
        const Vec3f ab = b - a;
        const Vec3f ac = c - a;
        const Vec3f ap = point - a;
        const float d1 = Dot(ab, ap);
        const float d2 = Dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f) return a;

        const Vec3f bp = point - b;
        const float d3 = Dot(ab, bp);
        const float d4 = Dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) return b;

        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            return a + ab * (d1 / (d1 - d3));

        const Vec3f cp = point - c;
        const float d5 = Dot(ab, cp);
        const float d6 = Dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) return c;

        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            return a + ac * (d2 / (d2 - d6));

        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
            return b + (c - b) * ((d4 - d3) /
                ((d4 - d3) + (d5 - d6)));

        const float denominator = 1.0f / (va + vb + vc);
        const float v = vb * denominator;
        const float w = vc * denominator;
        return a + ab * v + ac * w;
    }

    /// Input: a ray with normalized direction and a validated convex hull.
    /// Output: nearest nonnegative surface distance and outward normal.
    /// Task: clip the ray against every outward hull half-space; this is exact
    /// for the authored convex polytope and does not fall back to its AABB.
    [[nodiscard]] inline bool RaycastConvexHull(
        const RigidBody& body,
        const Collider& collider,
        const ConvexHullCollider& hull,
        const Vec3f& origin,
        const Vec3f& direction,
        float maxDistance,
        float& distance,
        Vec3f& normal)
    {
        const Vec3f center = WorldColliderCenter(body, collider);
        const Quaternionf rotation = WorldColliderRotation(body, collider);
        float entry = 0.0f;
        float exit = maxDistance;
        Vec3f entryNormal = -direction;
        for (const auto& face : hull.Faces)
        {
            const Vec3f a = center + Rotate(rotation, hull.Vertices[face[0]]);
            const Vec3f b = center + Rotate(rotation, hull.Vertices[face[1]]);
            const Vec3f c = center + Rotate(rotation, hull.Vertices[face[2]]);
            const Vec3f faceNormal = SafeNormalize(
                Cross(b - a, c - a), Vec3f::UnitX());
            const float originSide = Dot(faceNormal, origin - a);
            const float denominator = Dot(faceNormal, direction);
            if (std::abs(denominator) <= 1.0e-7f)
            {
                if (originSide > 0.0f) return false;
                continue;
            }
            const float time = -originSide / denominator;
            if (denominator < 0.0f)
            {
                if (time > entry)
                {
                    entry = time;
                    entryNormal = faceNormal;
                }
            }
            else exit = std::min(exit, time);
            if (entry > exit) return false;
        }
        if (entry < 0.0f || entry > maxDistance) return false;
        distance = entry;
        normal = entryNormal;
        return true;
    }

    /// Input: a world point and validated hull. Output: exact unsigned distance
    /// to the triangulated boundary plus an outward direction from hull to
    /// point; points inside return zero and the nearest face normal. Task:
    /// support conservative sphere sweeps against convex colliders.
    [[nodiscard]] inline float PointConvexHullSeparation(
        const RigidBody& body,
        const Collider& collider,
        const ConvexHullCollider& hull,
        const Vec3f& point,
        const Vec3f& fallbackNormal,
        Vec3f& normal)
    {
        const Vec3f center = WorldColliderCenter(body, collider);
        const Quaternionf rotation = WorldColliderRotation(body, collider);
        bool inside = true;
        float nearestInsidePlane = -std::numeric_limits<float>::infinity();
        Vec3f nearestInsideNormal = fallbackNormal;
        float bestDistanceSq = std::numeric_limits<float>::infinity();
        Vec3f bestDelta = fallbackNormal;
        for (const auto& face : hull.Faces)
        {
            const Vec3f a = center + Rotate(rotation, hull.Vertices[face[0]]);
            const Vec3f b = center + Rotate(rotation, hull.Vertices[face[1]]);
            const Vec3f c = center + Rotate(rotation, hull.Vertices[face[2]]);
            const Vec3f faceNormal = SafeNormalize(
                Cross(b - a, c - a), fallbackNormal);
            const float signedDistance = Dot(faceNormal, point - a);
            if (signedDistance > 1.0e-6f) inside = false;
            if (signedDistance > nearestInsidePlane)
            {
                nearestInsidePlane = signedDistance;
                nearestInsideNormal = faceNormal;
            }
            const Vec3f closest = ClosestPointOnConvexTriangle(point, a, b, c);
            const Vec3f delta = point - closest;
            const float distanceSq = delta.LengthSquared();
            if (distanceSq < bestDistanceSq)
            {
                bestDistanceSq = distanceSq;
                bestDelta = delta;
            }
        }
        if (inside)
        {
            normal = nearestInsideNormal;
            return 0.0f;
        }
        const float result = std::sqrt(std::max(0.0f, bestDistanceSq));
        normal = result > 1.0e-6f ? bestDelta / result : nearestInsideNormal;
        return result;
    }

    namespace convex_detail
    {
        constexpr float DirectionEpsilonSq = 1.0e-14f;

        [[nodiscard]] inline bool SameDirection(
            const Vec3f& direction,
            const Vec3f& toward) noexcept
        {
            return Dot(direction, toward) > 0.0f;
        }

        [[nodiscard]] inline Vec3f Perpendicular(const Vec3f& vector)
        {
            const Vec3f axis = std::abs(vector.x) < std::abs(vector.y)
                ? (std::abs(vector.x) < std::abs(vector.z)
                    ? Vec3f::UnitX() : Vec3f::UnitZ())
                : (std::abs(vector.y) < std::abs(vector.z)
                    ? Vec3f::UnitY() : Vec3f::UnitZ());
            return SafeNormalize(Cross(vector, axis), Vec3f::UnitX());
        }

        [[nodiscard]] inline Vec3f TripleCrossDirection(
            const Vec3f& edge,
            const Vec3f& toward)
        {
            const Vec3f result = Cross(Cross(edge, toward), edge);
            return result.LengthSquared() > DirectionEpsilonSq
                ? result : Perpendicular(edge);
        }

        inline bool HandleLine(ConvexSimplex& simplex, Vec3f& direction)
        {
            const Vec3f a = simplex.Points[0].Difference;
            const Vec3f b = simplex.Points[1].Difference;
            const Vec3f ab = b - a;
            const Vec3f ao = -a;
            if (SameDirection(ab, ao))
                direction = TripleCrossDirection(ab, ao);
            else
            {
                simplex.Count = 1u;
                direction = ao;
            }
            return direction.LengthSquared() <= DirectionEpsilonSq;
        }

        inline bool HandleTriangle(ConvexSimplex& simplex, Vec3f& direction)
        {
            const auto a = simplex.Points[0];
            const auto b = simplex.Points[1];
            const auto c = simplex.Points[2];
            const Vec3f ab = b.Difference - a.Difference;
            const Vec3f ac = c.Difference - a.Difference;
            const Vec3f ao = -a.Difference;
            const Vec3f abc = Cross(ab, ac);

            if (SameDirection(Cross(abc, ac), ao))
            {
                if (SameDirection(ac, ao))
                {
                    simplex.Points[1] = c;
                    simplex.Count = 2u;
                    direction = TripleCrossDirection(ac, ao);
                }
                else
                {
                    simplex.Points[1] = b;
                    simplex.Count = 2u;
                    return HandleLine(simplex, direction);
                }
                return false;
            }

            if (SameDirection(Cross(ab, abc), ao))
            {
                simplex.Points[1] = b;
                simplex.Count = 2u;
                return HandleLine(simplex, direction);
            }

            if (SameDirection(abc, ao))
                direction = abc;
            else
            {
                simplex.Points[1] = c;
                simplex.Points[2] = b;
                direction = -abc;
            }
            return direction.LengthSquared() <= DirectionEpsilonSq;
        }

        inline bool TestTetrahedronFace(
            ConvexSimplex& simplex,
            const ConvexSupportVertex& a,
            ConvexSupportVertex b,
            ConvexSupportVertex c,
            const ConvexSupportVertex& opposite,
            const Vec3f& ao,
            Vec3f& direction)
        {
            Vec3f normal = Cross(
                b.Difference - a.Difference,
                c.Difference - a.Difference);
            if (Dot(normal, opposite.Difference - a.Difference) > 0.0f)
            {
                std::swap(b, c);
                normal = -normal;
            }
            if (!SameDirection(normal, ao)) return false;
            simplex.Points[0] = a;
            simplex.Points[1] = b;
            simplex.Points[2] = c;
            simplex.Count = 3u;
            direction = normal;
            return true;
        }

        inline bool HandleTetrahedron(ConvexSimplex& simplex, Vec3f& direction)
        {
            const auto a = simplex.Points[0];
            const auto b = simplex.Points[1];
            const auto c = simplex.Points[2];
            const auto d = simplex.Points[3];
            const Vec3f ao = -a.Difference;
            if (TestTetrahedronFace(simplex, a, b, c, d, ao, direction) ||
                TestTetrahedronFace(simplex, a, c, d, b, ao, direction) ||
                TestTetrahedronFace(simplex, a, d, b, c, ao, direction))
                return HandleTriangle(simplex, direction);
            return true;
        }

        inline bool UpdateSimplex(ConvexSimplex& simplex, Vec3f& direction)
        {
            if (simplex.Count == 2u) return HandleLine(simplex, direction);
            if (simplex.Count == 3u) return HandleTriangle(simplex, direction);
            if (simplex.Count == 4u) return HandleTetrahedron(simplex, direction);
            direction = -simplex.Points[0].Difference;
            return direction.LengthSquared() <= DirectionEpsilonSq;
        }

        struct EPAFace final
        {
            std::array<std::uint32_t, 3> Indices{};
            Vec3f Normal = Vec3f::UnitX();
            float Distance = 0.0f;
        };

        [[nodiscard]] inline std::optional<EPAFace> MakeFace(
            const std::vector<ConvexSupportVertex>& vertices,
            std::uint32_t first,
            std::uint32_t second,
            std::uint32_t third)
        {
            Vec3f normal = Cross(
                vertices[second].Difference - vertices[first].Difference,
                vertices[third].Difference - vertices[first].Difference);
            if (normal.LengthSquared() <= DirectionEpsilonSq) return std::nullopt;
            normal = normal.Normalized();
            float distance = Dot(normal, vertices[first].Difference);
            if (distance < 0.0f)
            {
                std::swap(second, third);
                normal = -normal;
                distance = -distance;
            }
            return EPAFace{ { first, second, third }, normal, distance };
        }

        [[nodiscard]] inline std::array<float, 3> Barycentric(
            const Vec3f& point,
            const Vec3f& a,
            const Vec3f& b,
            const Vec3f& c)
        {
            const Vec3f v0 = b - a;
            const Vec3f v1 = c - a;
            const Vec3f v2 = point - a;
            const float d00 = Dot(v0, v0);
            const float d01 = Dot(v0, v1);
            const float d11 = Dot(v1, v1);
            const float d20 = Dot(v2, v0);
            const float d21 = Dot(v2, v1);
            const float denominator = d00 * d11 - d01 * d01;
            if (std::abs(denominator) <= 1.0e-12f)
                return { 1.0f, 0.0f, 0.0f };
            const float v = (d11 * d20 - d01 * d21) / denominator;
            const float w = (d00 * d21 - d01 * d20) / denominator;
            return { 1.0f - v - w, v, w };
        }

        inline void AddBoundaryEdge(
            std::vector<std::pair<std::uint32_t, std::uint32_t>>& edges,
            std::uint32_t first,
            std::uint32_t second)
        {
            const auto reverse = std::ranges::find(
                edges, std::pair{ second, first });
            if (reverse != edges.end()) edges.erase(reverse);
            else edges.emplace_back(first, second);
        }
    }

    /// Input: two validated finite convex colliders. Output: a solver-ready
    /// penetration normal (A toward B), depth, and witness midpoint, or no
    /// value when separated/degenerate. Task: use GJK for intersection and EPA
    /// for penetration without depending on render topology or allocation in
    /// the hot support operation. Iteration caps make failure deterministic.
    [[nodiscard]] inline std::optional<ConvexPenetration> CollideConvex(
        const RigidBody& bodyA,
        const Collider& colliderA,
        const RigidBody& bodyB,
        const Collider& colliderB)
    {
        if (IsInfiniteCollider(colliderA) || IsInfiniteCollider(colliderB))
            throw std::invalid_argument(
                "CollideConvex requires two finite support-map colliders.");

        Vec3f direction = WorldColliderCenter(bodyB, colliderB) -
            WorldColliderCenter(bodyA, colliderA);
        if (direction.LengthSquared() <= convex_detail::DirectionEpsilonSq)
            direction = Vec3f::UnitX();
        const float directionScale = std::max(1.0f, direction.Length());
        direction += Vec3f{ 0.013f, 0.017f, 0.019f } * directionScale;

        ConvexSimplex simplex;
        simplex.PushFront(ConvexMinkowskiSupport(
            bodyA, colliderA, bodyB, colliderB, direction));
        direction = -simplex.Points[0].Difference;
        std::uint32_t gjkIterations = 0u;
        bool intersects = false;
        for (; gjkIterations < 32u; ++gjkIterations)
        {
            if (direction.LengthSquared() <= convex_detail::DirectionEpsilonSq)
            {
                intersects = simplex.Count == 4u;
                break;
            }
            const auto support = ConvexMinkowskiSupport(
                bodyA, colliderA, bodyB, colliderB, direction);
            if (Dot(support.Difference, direction) <= 1.0e-6f)
                return std::nullopt;
            bool duplicate = false;
            for (std::size_t index = 0u; index < simplex.Count; ++index)
                duplicate = duplicate ||
                    (simplex.Points[index].Difference - support.Difference)
                        .LengthSquared() <= 1.0e-12f;
            if (duplicate) return std::nullopt;
            simplex.PushFront(support);
            if (convex_detail::UpdateSimplex(simplex, direction))
            {
                intersects = simplex.Count == 4u;
                break;
            }
        }
        if (!intersects || simplex.Count != 4u) return std::nullopt;

        std::vector<ConvexSupportVertex> vertices(
            simplex.Points.begin(), simplex.Points.begin() + 4u);
        std::vector<convex_detail::EPAFace> faces;
        for (const auto indices : std::array<std::array<std::uint32_t, 3>, 4>{
            std::array<std::uint32_t, 3>{ 0u, 1u, 2u },
            std::array<std::uint32_t, 3>{ 0u, 3u, 1u },
            std::array<std::uint32_t, 3>{ 0u, 2u, 3u },
            std::array<std::uint32_t, 3>{ 1u, 3u, 2u } })
            if (const auto face = convex_detail::MakeFace(
                vertices, indices[0], indices[1], indices[2]))
                faces.push_back(*face);
        if (faces.size() != 4u) return std::nullopt;

        for (std::uint32_t epaIteration = 0u; epaIteration < 64u; ++epaIteration)
        {
            const auto closest = std::ranges::min_element(
                faces, {}, &convex_detail::EPAFace::Distance);
            if (closest == faces.end()) return std::nullopt;
            const auto support = ConvexMinkowskiSupport(
                bodyA, colliderA, bodyB, colliderB, closest->Normal);
            const float supportDistance =
                Dot(support.Difference, closest->Normal);
            if (supportDistance - closest->Distance <= 1.0e-4f)
            {
                Vec3f normal = closest->Normal;
                const Vec3f centerDelta = WorldColliderCenter(bodyB, colliderB) -
                    WorldColliderCenter(bodyA, colliderA);
                if (Dot(normal, centerDelta) < 0.0f) normal = -normal;
                const auto& va = vertices[closest->Indices[0]];
                const auto& vb = vertices[closest->Indices[1]];
                const auto& vc = vertices[closest->Indices[2]];
                const auto weights = convex_detail::Barycentric(
                    closest->Normal * closest->Distance,
                    va.Difference, vb.Difference, vc.Difference);
                const Vec3f witnessA = va.PointA * weights[0] +
                    vb.PointA * weights[1] + vc.PointA * weights[2];
                const Vec3f witnessB = va.PointB * weights[0] +
                    vb.PointB * weights[1] + vc.PointB * weights[2];
                return ConvexPenetration{ (witnessA + witnessB) * 0.5f,
                    normal, closest->Distance, gjkIterations + 1u,
                    epaIteration + 1u };
            }

            bool duplicate = false;
            for (const auto& vertex : vertices)
                duplicate = duplicate ||
                    (vertex.Difference - support.Difference).LengthSquared()
                        <= 1.0e-12f;
            if (duplicate) return std::nullopt;
            const auto newIndex = static_cast<std::uint32_t>(vertices.size());
            vertices.push_back(support);
            std::vector<std::pair<std::uint32_t, std::uint32_t>> boundary;
            for (std::size_t index = faces.size(); index-- > 0u;)
            {
                const auto& face = faces[index];
                if (Dot(face.Normal, support.Difference -
                    vertices[face.Indices[0]].Difference) <= 1.0e-6f)
                    continue;
                convex_detail::AddBoundaryEdge(
                    boundary, face.Indices[0], face.Indices[1]);
                convex_detail::AddBoundaryEdge(
                    boundary, face.Indices[1], face.Indices[2]);
                convex_detail::AddBoundaryEdge(
                    boundary, face.Indices[2], face.Indices[0]);
                faces.erase(faces.begin() + static_cast<std::ptrdiff_t>(index));
            }
            if (boundary.empty()) return std::nullopt;
            for (const auto [first, second] : boundary)
                if (const auto face = convex_detail::MakeFace(
                    vertices, first, second, newIndex))
                    faces.push_back(*face);
        }
        return std::nullopt;
    }
}
