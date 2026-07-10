module;

#include <algorithm>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Broadphase;

import Kairo.Foundation.Spatial;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;

export namespace kairo::foundation::physics
{
    namespace spatial = kairo::foundation::spatial;

    struct BroadphasePair final
    {
        ColliderID A = InvalidColliderID;
        ColliderID B = InvalidColliderID;

        [[nodiscard]]
        friend constexpr bool operator==(
            const BroadphasePair& lhs,
            const BroadphasePair& rhs) noexcept = default;
    };

    [[nodiscard]]
    inline BroadphasePair OrderedBroadphasePair(
        ColliderID a,
        ColliderID b) noexcept
    {
        return a < b
            ? BroadphasePair{ a, b }
            : BroadphasePair{ b, a };
    }

    /// Input: bodies and colliders.
    /// Output: deterministic potentially-overlapping collider pairs.
    /// Task: reuse KairoSpatial's dynamic AABB tree for finite collider overlap
    /// queries while pairing infinite planes against all finite colliders.
    [[nodiscard]]
    inline std::vector<BroadphasePair> ComputeBroadphasePairs(
        const std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders)
    {
        spatial::DynamicAABBTree tree;
        std::vector<ColliderID> finiteColliders;
        std::vector<ColliderID> planeColliders;

        for (const Collider& collider : colliders)
        {
            if (collider.Body >= bodies.size())
            {
                continue;
            }

            if (IsInfiniteCollider(collider))
            {
                planeColliders.push_back(collider.ID);
                continue;
            }

            [[maybe_unused]] const spatial::SpatialIndex proxy =
                tree.Insert(
                collider.ID,
                WorldAABB(bodies.at(collider.Body), collider),
                collider.LayerMask);

            finiteColliders.push_back(collider.ID);
        }

        std::vector<BroadphasePair> pairs;
        for (const spatial::SpatialPair& pair : tree.ComputePairs())
        {
            pairs.push_back(OrderedBroadphasePair(pair.A, pair.B));
        }

        for (ColliderID finite : finiteColliders)
        {
            for (ColliderID plane : planeColliders)
            {
                pairs.push_back(OrderedBroadphasePair(finite, plane));
            }
        }

        std::sort(
            pairs.begin(),
            pairs.end(),
            [](const BroadphasePair& lhs, const BroadphasePair& rhs)
            {
                return lhs.A == rhs.A ? lhs.B < rhs.B : lhs.A < rhs.A;
            });

        pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
        return pairs;
    }
}
