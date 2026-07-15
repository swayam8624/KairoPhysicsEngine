module;

#include <cstdint>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Debug;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;
    using namespace kairo::foundation::geometry;

    struct DebugContact final
    {
        Vec3f Position = Vec3f::Zero();
        Vec3f Normal = Vec3f::Up();
        float Penetration = 0.0f;
    };

    struct DebugAABB final
    {
        ColliderID Collider = InvalidColliderID;
        AABBf Bounds = AABBf::Empty();
    };

    /// Shape vocabulary intentionally mirrors the physics collider variant.
    /// Rendering code should branch on this stable debug contract rather than
    /// inspecting `ColliderShape` or retaining references into PhysicsWorld.
    enum class DebugShapeKind : std::uint8_t
    {
        Sphere,
        Capsule,
        Plane,
        AABB,
        Box
    };

    /// Input: an active collider and its owner transform.
    /// Output: an immutable world-space draw description.
    /// Task: make a future renderer capable of drawing actual collider shapes,
    /// not merely their broadphase bounds. `Center`, `Rotation`, `Radius`, and
    /// `HalfExtents` are meaningful for their corresponding `Kind`; capsule
    /// endpoints are supplied directly to avoid divergent local-Y conventions.
    struct DebugShape final
    {
        BodyID Body = InvalidBodyID;
        ColliderID Collider = InvalidColliderID;
        DebugShapeKind Kind = DebugShapeKind::Sphere;
        Vec3f Center = Vec3f::Zero();
        Quaternionf Rotation = Quaternionf::Identity();
        Vec3f HalfExtents = Vec3f::Zero();
        Vec3f SegmentStart = Vec3f::Zero();
        Vec3f SegmentEnd = Vec3f::Zero();
        Vec3f PlaneNormal = Vec3f::Up();
        float PlaneDistance = 0.0f;
        float Radius = 0.0f;
        bool IsTrigger = false;
        bool Sleeping = false;
    };

    /// Input: bodies and colliders.
    /// Output: finite collider AABBs for debug drawing or logging.
    /// Task: expose broadphase bounds without coupling to a renderer.
    [[nodiscard]]
    inline std::vector<DebugAABB> CollectDebugAABBs(
        const std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders)
    {
        std::vector<DebugAABB> result;
        for (const Collider& collider : colliders)
        {
            if (IsActiveCollider(collider) &&
                collider.Body < bodies.size() &&
                IsActiveBody(bodies.at(collider.Body)) &&
                !IsInfiniteCollider(collider))
            {
                result.push_back(
                    DebugAABB
                    {
                        collider.ID,
                        WorldAABB(bodies.at(collider.Body), collider)
                    });
            }
        }

        return result;
    }

    /// Input: contact manifolds.
    /// Output: flattened debug contacts.
    /// Task: provide renderer-agnostic contact data for terminal/window tools.
    [[nodiscard]]
    inline std::vector<DebugContact> CollectDebugContacts(
        const std::vector<ContactManifold>& contacts)
    {
        std::vector<DebugContact> result;
        for (const ContactManifold& manifold : contacts)
        {
            for (const ContactPoint& point : manifold.Points)
            {
                result.push_back(
                    DebugContact
                    {
                        point.Position,
                        point.Normal,
                        point.PenetrationDepth
                    });
            }
        }

        return result;
    }

    /// Input: bodies and colliders owned by a PhysicsWorld.
    /// Output: one deterministic record per active collider, including planes.
    /// Task: bridge physics state to visualizers without OpenGL, GLFW, a scene
    /// format, or any renderer dependency. The records are values, so callers
    /// may cache them across frames safely.
    [[nodiscard]]
    inline std::vector<DebugShape> CollectDebugShapes(
        const std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders)
    {
        std::vector<DebugShape> result;
        result.reserve(colliders.size());

        for (const Collider& collider : colliders)
        {
            if (!IsActiveCollider(collider) ||
                collider.Body >= bodies.size() ||
                !IsActiveBody(bodies.at(collider.Body)))
            {
                continue;
            }

            const RigidBody& body = bodies.at(collider.Body);
            DebugShape shape;
            shape.Body = body.ID;
            shape.Collider = collider.ID;
            shape.Center = WorldColliderCenter(body, collider);
            shape.Rotation = WorldColliderRotation(body, collider);
            shape.IsTrigger = collider.IsTrigger;
            shape.Sleeping = body.Sleeping;

            if (const auto* sphere = std::get_if<SphereCollider>(&collider.Shape))
            {
                shape.Kind = DebugShapeKind::Sphere;
                shape.Radius = sphere->Radius;
            }
            else if (const auto* capsule = std::get_if<CapsuleCollider>(&collider.Shape))
            {
                const CapsuleSegment segment = WorldCapsuleSegment(body, collider, *capsule);
                shape.Kind = DebugShapeKind::Capsule;
                shape.Radius = capsule->Radius;
                shape.SegmentStart = segment.A;
                shape.SegmentEnd = segment.B;
            }
            else if (const auto* plane = std::get_if<PlaneCollider>(&collider.Shape))
            {
                shape.Kind = DebugShapeKind::Plane;
                shape.PlaneNormal = plane->Normal;
                shape.PlaneDistance = plane->Distance;
            }
            else if (const auto* aabb = std::get_if<AABBCollider>(&collider.Shape))
            {
                shape.Kind = DebugShapeKind::AABB;
                shape.HalfExtents = aabb->HalfExtents;
            }
            else if (const auto* box = std::get_if<BoxCollider>(&collider.Shape))
            {
                shape.Kind = DebugShapeKind::Box;
                shape.HalfExtents = box->HalfExtents;
            }

            result.push_back(shape);
        }

        return result;
    }
}
