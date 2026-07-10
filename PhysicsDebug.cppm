module;

#include <vector>

export module Kairo.Foundation.PhysicsEngine.Debug;

import Kairo.Foundation.Math.Vector;
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
}
