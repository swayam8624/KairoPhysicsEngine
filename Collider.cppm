module;

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Collider;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.Geometry.Sphere;
import Kairo.Foundation.Geometry.Plane;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Spatial.Types;
import Kairo.Foundation.Spatial.BVH;
import Kairo.Foundation.Spatial.BVHBuilder;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Material;
import Kairo.Foundation.PhysicsEngine.RigidBody;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;
    using namespace kairo::foundation::geometry;

    /// Game-engine style response for a candidate collider pair.
    ///
    /// Input: two colliders selected by broadphase/narrowphase.
    /// Output: whether that pair is ignored, reported as a trigger, or solved.
    /// Task: separate "what objects can see each other" from "what the solver
    /// should do with the contact." This is the same idea as collision response
    /// channels in commercial engines: a player can block terrain, overlap a
    /// pickup trigger, and ignore its own sensor rays with the same collider API.
    enum class CollisionResponse : std::uint8_t
    {
        Ignore,
        Trigger,
        Block
    };

    /// Built-in layer bits for common gameplay categories.
    ///
    /// Input: none.
    /// Output: stable 32-bit category masks that callers can combine or replace.
    /// Task: provide useful defaults without locking the engine into a fixed
    /// gameplay taxonomy. Users can still pass custom `1u << n` masks directly.
    struct CollisionLayer final
    {
        static constexpr std::uint32_t Default = 1u << 0u;
        static constexpr std::uint32_t StaticWorld = 1u << 1u;
        static constexpr std::uint32_t DynamicWorld = 1u << 2u;
        static constexpr std::uint32_t Player = 1u << 3u;
        static constexpr std::uint32_t Character = 1u << 4u;
        static constexpr std::uint32_t Projectile = 1u << 5u;
        static constexpr std::uint32_t Trigger = 1u << 6u;
        static constexpr std::uint32_t Sensor = 1u << 7u;
        static constexpr std::uint32_t Cloth = 1u << 8u;
        static constexpr std::uint32_t Fluid = 1u << 9u;
        static constexpr std::uint32_t Particle = 1u << 10u;
        static constexpr std::uint32_t All = 0xFFFF'FFFFu;
    };

    struct SphereCollider final
    {
        float Radius = 0.5f;
    };

    /// A capsule aligned to its local Y axis.
    ///
    /// `HalfHeight` is the half-length of the straight center segment, not the
    /// half of the total rounded height. Its total world-space height is
    /// `2 * (HalfHeight + Radius)`. This convention keeps a zero-height capsule
    /// equivalent to a sphere while remaining convenient for character shapes.
    struct CapsuleCollider final
    {
        float Radius = 0.5f;
        float HalfHeight = 0.5f;
    };

    struct PlaneCollider final
    {
        Vec3f Normal = Vec3f::Up();
        float Distance = 0.0f;
    };

    struct AABBCollider final
    {
        Vec3f HalfExtents = Vec3f{ 0.5f, 0.5f, 0.5f };
    };

    struct BoxCollider final
    {
        Vec3f HalfExtents = Vec3f{ 0.5f, 0.5f, 0.5f };
    };

    /// Closed, triangulated convex collision hull in collider-local space.
    ///
    /// Input: at least four non-coplanar vertices and outward/inward triangle
    /// faces. `MakeCollider` normalizes face winding to outward and rejects
    /// open, non-manifold, degenerate, or concave input. Output: a support-map
    /// shape suitable for GJK/EPA and exact plane/ray tests. Task: represent
    /// authored convex collision geometry without retaining render meshes or
    /// importing a third-party collision library.
    struct ConvexHullCollider final
    {
        std::vector<Vec3f> Vertices;
        std::vector<std::array<std::uint32_t, 3>> Faces;
    };

    /// Concave triangle collision surface accelerated in collider-local space.
    ///
    /// Vertices and triangle indices are immutable authored geometry after the
    /// collider is created. `Acceleration` stores one KairoSpatial BVH primitive
    /// per triangle; because that tree is local-space, rigid world transforms do
    /// not require rebuilding it. `SurfaceThickness` turns the mathematically
    /// zero-volume triangles into a thin collision shell for robust GJK/EPA
    /// contact generation while ray and sweep tests remain triangle-exact.
    struct TriangleMeshCollider final
    {
        std::vector<Vec3f> Vertices;
        std::vector<std::array<std::uint32_t, 3>> Triangles;
        kairo::foundation::spatial::BVH Acceleration;
        float SurfaceThickness = 1.0e-3f;
        bool DoubleSided = true;
    };

    using ColliderShape =
        std::variant<SphereCollider, CapsuleCollider, PlaneCollider, AABBCollider,
            BoxCollider, ConvexHullCollider, TriangleMeshCollider>;

    struct Collider final
    {
        ColliderID ID = InvalidColliderID;
        bool Active = true;
        BodyID Body = InvalidBodyID;
        Vec3f LocalCenter = Vec3f::Zero();
        Quaternionf LocalRotation = Quaternionf::Identity();
        ColliderShape Shape = SphereCollider{};
        PhysicsMaterial Material;
        std::uint32_t BelongsTo = 1u;
        std::uint32_t CollidesWith = 0xFFFF'FFFFu;
        bool IsTrigger = false;
        std::uint32_t LayerMask = 1u;
    };

    /// Input: collider.
    /// Output: true when the collider is infinite and should not enter an AABB tree.
    /// Task: planes are tested against finite colliders explicitly by broadphase.
    [[nodiscard]]
    inline bool IsInfiniteCollider(
        const Collider& collider) noexcept
    {
        return collider.Active &&
            std::holds_alternative<PlaneCollider>(collider.Shape);
    }

    /// Input: collider.
    /// Output: true when the collider still participates in simulation.
    /// Task: make collider ids deletion-safe without erasing vector storage or
    /// changing ids held by user code, debug tools, or contact caches.
    [[nodiscard]]
    inline bool IsActiveCollider(
        const Collider& collider) noexcept
    {
        return collider.Active &&
            collider.ID != InvalidColliderID &&
            collider.Body != InvalidBodyID;
    }

    /// Input: two colliders.
    /// Output: true when their category masks allow narrowphase testing.
    /// Task: implement the standard "belongs to / collides with" filter while
    /// keeping the older `LayerMask` field as a broadphase category hint.
    [[nodiscard]]
    inline bool CollisionFiltersAllow(
        const Collider& a,
        const Collider& b) noexcept
    {
        return (a.CollidesWith & b.BelongsTo) != 0u &&
            (b.CollidesWith & a.BelongsTo) != 0u;
    }

    /// Input: two colliders after coarse masks allow them to be considered.
    /// Output: default response before world-level override rules/callbacks.
    /// Task: preserve the existing V1 behavior: masks ignore, trigger colliders
    /// report overlap only, and all other allowed pairs block/solve.
    [[nodiscard]]
    inline CollisionResponse DefaultCollisionResponse(
        const Collider& a,
        const Collider& b) noexcept
    {
        if (!CollisionFiltersAllow(a, b))
        {
            return CollisionResponse::Ignore;
        }

        if (a.IsTrigger || b.IsTrigger)
        {
            return CollisionResponse::Trigger;
        }

        return CollisionResponse::Block;
    }

    /// Input: collider.
    /// Output: broadphase category mask passed to KairoSpatial.
    /// Task: prefer the explicit category field while preserving V1 LayerMask
    /// compatibility for callers that still mutate it directly.
    [[nodiscard]]
    inline std::uint32_t BroadphaseCategoryMask(
        const Collider& collider) noexcept
    {
        return collider.BelongsTo != 0u ? collider.BelongsTo : collider.LayerMask;
    }

    /// Input: body and collider.
    /// Output: collider center in world coordinates.
    /// Task: apply V1 local-center offset. Local collider rotation is deferred.
    [[nodiscard]]
    inline Vec3f WorldColliderCenter(
        const RigidBody& body,
        const Collider& collider)
    {
        return body.State.Position + Rotate(body.State.Rotation, collider.LocalCenter);
    }

    /// Input: body and collider.
    /// Output: world-space collider orientation.
    /// Task: combine body orientation with local collider rotation so shapes can
    /// be authored as rotated child primitives without creating extra bodies.
    [[nodiscard]]
    inline Quaternionf WorldColliderRotation(
        const RigidBody& body,
        const Collider& collider)
    {
        return (body.State.Rotation * collider.LocalRotation).Normalized();
    }

    struct OrientedBoxFrame final
    {
        Vec3f Center = Vec3f::Zero();
        std::array<Vec3f, 3> Axes
        {
            Vec3f::UnitX(),
            Vec3f::UnitY(),
            Vec3f::UnitZ()
        };
        Vec3f HalfExtents = Vec3f{ 0.5f, 0.5f, 0.5f };
    };

    struct CapsuleSegment final
    {
        Vec3f A = Vec3f::Zero();
        Vec3f B = Vec3f::Zero();
        float Radius = 0.5f;
    };

    /// Input: owning body, capsule collider, and optional child transform.
    /// Output: world-space endpoints of the capsule's straight center segment.
    /// Task: centralize the local-Y convention used by collision, ray, sweep,
    /// debug, and future character-controller code paths.
    [[nodiscard]]
    inline CapsuleSegment WorldCapsuleSegment(
        const RigidBody& body,
        const Collider& collider,
        const CapsuleCollider& capsule)
    {
        const Vec3f center = WorldColliderCenter(body, collider);
        const Vec3f axis = SafeNormalize(
            Rotate(WorldColliderRotation(body, collider), Vec3f::UnitY()),
            Vec3f::UnitY());
        const Vec3f halfSegment = axis * capsule.HalfHeight;
        return { center - halfSegment, center + halfSegment, capsule.Radius };
    }

    /// Input: body, collider, and box half extents.
    /// Output: OBB center, orthonormal axes, and extents in world space.
    /// Task: prepare BoxCollider data for broadphase bounds and SAT narrowphase.
    [[nodiscard]]
    inline OrientedBoxFrame WorldBoxFrame(
        const RigidBody& body,
        const Collider& collider,
        const Vec3f& halfExtents)
    {
        const Quaternionf rotation =
            WorldColliderRotation(body, collider);

        return
        {
            WorldColliderCenter(body, collider),
            {
                SafeNormalize(Rotate(rotation, Vec3f::UnitX()), Vec3f::UnitX()),
                SafeNormalize(Rotate(rotation, Vec3f::UnitY()), Vec3f::UnitY()),
                SafeNormalize(Rotate(rotation, Vec3f::UnitZ()), Vec3f::UnitZ())
            },
            halfExtents
        };
    }

    /// Input: body and collider.
    /// Output: world-space AABB for finite collider shapes.
    /// Task: feed KairoSpatial broadphase with bounds derived from engine data.
    [[nodiscard]]
    inline AABBf WorldAABB(
        const RigidBody& body,
        const Collider& collider)
    {
        const Vec3f center =
            WorldColliderCenter(body, collider);

        if (const auto* sphere = std::get_if<SphereCollider>(&collider.Shape))
        {
            RequirePositive(sphere->Radius, "SphereCollider.Radius");
            return AABBf::FromCenterExtent(
                center,
                Vec3f{ sphere->Radius, sphere->Radius, sphere->Radius });
        }

        if (const auto* capsule = std::get_if<CapsuleCollider>(&collider.Shape))
        {
            RequirePositive(capsule->Radius, "CapsuleCollider.Radius");
            RequireNonNegative(capsule->HalfHeight, "CapsuleCollider.HalfHeight");

            const CapsuleSegment segment =
                WorldCapsuleSegment(body, collider, *capsule);
            const Vec3f extents =
                Abs((segment.B - segment.A) * 0.5f) +
                Vec3f{ capsule->Radius, capsule->Radius, capsule->Radius };
            return AABBf::FromCenterExtent((segment.A + segment.B) * 0.5f, extents);
        }

        if (const auto* box = std::get_if<AABBCollider>(&collider.Shape))
        {
            RequirePositiveComponents(box->HalfExtents, "AABBCollider.HalfExtents");
            return AABBf::FromCenterExtent(center, box->HalfExtents);
        }

        if (const auto* box = std::get_if<BoxCollider>(&collider.Shape))
        {
            RequirePositiveComponents(box->HalfExtents, "BoxCollider.HalfExtents");

            const OrientedBoxFrame frame =
                WorldBoxFrame(body, collider, box->HalfExtents);

            const Vec3f extents =
                Abs(frame.Axes[0]) * frame.HalfExtents.x +
                Abs(frame.Axes[1]) * frame.HalfExtents.y +
                Abs(frame.Axes[2]) * frame.HalfExtents.z;

            return AABBf::FromCenterExtent(frame.Center, extents);
        }

        if (const auto* hull = std::get_if<ConvexHullCollider>(&collider.Shape))
        {
            AABBf bounds = AABBf::Empty();
            const Quaternionf rotation = WorldColliderRotation(body, collider);
            for (const Vec3f& vertex : hull->Vertices)
            {
                bounds.ExpandToInclude(center + Rotate(rotation, vertex));
            }
            return bounds;
        }

        if (const auto* mesh = std::get_if<TriangleMeshCollider>(&collider.Shape))
        {
            if (mesh->Acceleration.Empty()) return AABBf::Empty();
            const AABBf local = mesh->Acceleration.Root().Bounds;
            const Quaternionf rotation = WorldColliderRotation(body, collider);
            AABBf bounds = AABBf::Empty();
            for (std::uint32_t mask = 0u; mask < 8u; ++mask)
            {
                const Vec3f corner
                {
                    (mask & 1u) != 0u ? local.Max.x : local.Min.x,
                    (mask & 2u) != 0u ? local.Max.y : local.Min.y,
                    (mask & 4u) != 0u ? local.Max.z : local.Min.z
                };
                bounds.ExpandToInclude(center + Rotate(rotation, corner));
            }
            return bounds;
        }

        return AABBf::Empty();
    }

    /// Input: collider id, body id, shape, material, and optional local center.
    /// Output: validated collider record.
    /// Task: provide a single construction path for `PhysicsWorld::AddCollider`.
    [[nodiscard]]
    inline Collider MakeCollider(
        ColliderID id,
        BodyID body,
        ColliderShape shape,
        PhysicsMaterial material = {},
        const Vec3f& localCenter = Vec3f::Zero(),
        const Quaternionf& localRotation = Quaternionf::Identity())
    {
        ValidatePhysicsMaterial(material);
        RequireFinite(localCenter, "localCenter");
        RequireFinite(localRotation.x, "localRotation.x");
        RequireFinite(localRotation.y, "localRotation.y");
        RequireFinite(localRotation.z, "localRotation.z");
        RequireFinite(localRotation.w, "localRotation.w");

        if (const auto* sphere = std::get_if<SphereCollider>(&shape))
        {
            RequirePositive(sphere->Radius, "SphereCollider.Radius");
        }
        else if (const auto* capsule = std::get_if<CapsuleCollider>(&shape))
        {
            RequirePositive(capsule->Radius, "CapsuleCollider.Radius");
            RequireNonNegative(capsule->HalfHeight, "CapsuleCollider.HalfHeight");
        }
        else if (const auto* box = std::get_if<AABBCollider>(&shape))
        {
            RequirePositiveComponents(box->HalfExtents, "AABBCollider.HalfExtents");
        }
        else if (const auto* box = std::get_if<BoxCollider>(&shape))
        {
            RequirePositiveComponents(box->HalfExtents, "BoxCollider.HalfExtents");
        }
        else if (auto* hull = std::get_if<ConvexHullCollider>(&shape))
        {
            if (hull->Vertices.size() < 4u || hull->Faces.size() < 4u)
                throw std::invalid_argument(
                    "ConvexHullCollider requires at least four vertices and faces.");

            Vec3f centroid = Vec3f::Zero();
            float scale = 0.0f;
            for (const Vec3f& vertex : hull->Vertices)
            {
                RequireFinite(vertex, "ConvexHullCollider.Vertex");
                centroid += vertex;
                scale = std::max(scale, vertex.Length());
            }
            centroid /= static_cast<float>(hull->Vertices.size());
            const float tolerance = std::max(1.0f, scale) * 1.0e-5f;
            for (std::size_t first = 0u; first < hull->Vertices.size(); ++first)
                for (std::size_t second = first + 1u;
                    second < hull->Vertices.size(); ++second)
                    if ((hull->Vertices[first] - hull->Vertices[second])
                        .LengthSquared() <= tolerance * tolerance)
                        throw std::invalid_argument(
                            "ConvexHullCollider contains duplicate vertices.");
            std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t>
                edgeUse;
            std::vector<std::uint32_t> vertexUse(hull->Vertices.size(), 0u);
            std::set<std::array<std::uint32_t, 3>> uniqueFaces;

            for (auto& face : hull->Faces)
            {
                if (face[0] >= hull->Vertices.size() ||
                    face[1] >= hull->Vertices.size() ||
                    face[2] >= hull->Vertices.size() ||
                    face[0] == face[1] || face[1] == face[2] ||
                    face[2] == face[0])
                    throw std::invalid_argument(
                        "ConvexHullCollider face contains invalid vertex indices.");

                auto faceKey = face;
                std::sort(faceKey.begin(), faceKey.end());
                if (!uniqueFaces.insert(faceKey).second)
                    throw std::invalid_argument(
                        "ConvexHullCollider contains duplicate faces.");
                ++vertexUse[face[0]];
                ++vertexUse[face[1]];
                ++vertexUse[face[2]];

                const Vec3f& a = hull->Vertices[face[0]];
                const Vec3f& b = hull->Vertices[face[1]];
                const Vec3f& c = hull->Vertices[face[2]];
                Vec3f normal = Cross(b - a, c - a);
                if (normal.LengthSquared() <= tolerance * tolerance)
                    throw std::invalid_argument(
                        "ConvexHullCollider contains a degenerate face.");
                if (Dot(normal, centroid - a) > 0.0f)
                {
                    std::swap(face[1], face[2]);
                    normal = -normal;
                }
                normal = normal.Normalized();
                for (const Vec3f& vertex : hull->Vertices)
                    if (Dot(normal, vertex - a) > tolerance)
                        throw std::invalid_argument(
                            "ConvexHullCollider vertices do not form a convex hull.");

                for (std::size_t edge = 0u; edge < 3u; ++edge)
                {
                    const auto first = face[edge];
                    const auto second = face[(edge + 1u) % 3u];
                    ++edgeUse[std::minmax(first, second)];
                }
            }
            for (const auto& [edge, count] : edgeUse)
                if (count != 2u)
                    throw std::invalid_argument(
                        "ConvexHullCollider must be a closed two-manifold mesh.");
            if (std::find(vertexUse.begin(), vertexUse.end(), 0u) !=
                vertexUse.end())
                throw std::invalid_argument(
                    "ConvexHullCollider contains an unused vertex.");
        }
        else if (auto* mesh = std::get_if<TriangleMeshCollider>(&shape))
        {
            using namespace kairo::foundation::spatial;
            if (mesh->Vertices.size() < 3u || mesh->Triangles.empty())
                throw std::invalid_argument(
                    "TriangleMeshCollider requires vertices and at least one triangle.");
            RequirePositive(mesh->SurfaceThickness,
                "TriangleMeshCollider.SurfaceThickness");

            for (const Vec3f& vertex : mesh->Vertices)
                RequireFinite(vertex, "TriangleMeshCollider.Vertex");

            std::vector<SpatialPrimitive> primitives;
            primitives.reserve(mesh->Triangles.size());
            for (std::size_t index = 0u; index < mesh->Triangles.size(); ++index)
            {
                const auto& triangle = mesh->Triangles[index];
                if (triangle[0] >= mesh->Vertices.size() ||
                    triangle[1] >= mesh->Vertices.size() ||
                    triangle[2] >= mesh->Vertices.size() ||
                    triangle[0] == triangle[1] || triangle[1] == triangle[2] ||
                    triangle[2] == triangle[0])
                    throw std::invalid_argument(
                        "TriangleMeshCollider triangle contains invalid vertex indices.");

                const Vec3f& a = mesh->Vertices[triangle[0]];
                const Vec3f& b = mesh->Vertices[triangle[1]];
                const Vec3f& c = mesh->Vertices[triangle[2]];
                if (Cross(b - a, c - a).LengthSquared() <= 1.0e-12f)
                    throw std::invalid_argument(
                        "TriangleMeshCollider contains a degenerate triangle.");

                AABBf bounds = AABBf::Empty();
                bounds.ExpandToInclude(a);
                bounds.ExpandToInclude(b);
                bounds.ExpandToInclude(c);
                const Vec3f margin{ mesh->SurfaceThickness, mesh->SurfaceThickness,
                    mesh->SurfaceThickness };
                bounds = AABBf::FromMinMax(bounds.Min - margin, bounds.Max + margin);
                primitives.push_back({
                    static_cast<SpatialID>(index), bounds, CollisionLayer::All });
            }

            BVHBuildSettings settings;
            settings.MaxLeafSize = 4u;
            settings.MaxDepth = 64u;
            settings.UseSAH = true;
            mesh->Acceleration = BuildBVH(primitives, settings);
            if (!mesh->Acceleration.IsValid())
                throw std::runtime_error(
                    "TriangleMeshCollider failed to build a valid acceleration BVH.");
        }
        else if (auto* plane = std::get_if<PlaneCollider>(&shape))
        {
            RequireFinite(plane->Normal, "PlaneCollider.Normal");
            RequireFinite(plane->Distance, "PlaneCollider.Distance");
            plane->Normal = SafeNormalize(plane->Normal, Vec3f::Up());
        }

        return { id, true, body, localCenter, localRotation.Normalized(),
            std::move(shape), material, 1u, 0xFFFF'FFFFu, false, 1u };
    }
}
