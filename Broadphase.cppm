module;

#include <algorithm>
#include <cstdint>
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

    struct BroadphaseProxy final
    {
        spatial::SpatialIndex Proxy = spatial::SpatialInvalidIndex;
        std::uint32_t CategoryMask = 0u;
        bool InTree = false;
        bool Infinite = false;
    };

    class BroadphaseWorld final
    {
    public:
        void Clear()
        {
            m_Tree = spatial::DynamicAABBTree{};
            m_Proxies.clear();
            m_InfiniteColliders.clear();
        }

        void AddOrUpdateCollider(
            const std::vector<RigidBody>& bodies,
            const Collider& collider)
        {
            EnsureProxy(collider.ID);

            BroadphaseProxy& proxy =
                m_Proxies.at(collider.ID);

            if (!IsActiveCollider(collider) ||
                collider.Body >= bodies.size() ||
                !IsActiveBody(bodies.at(collider.Body)))
            {
                RemoveCollider(collider.ID);
                return;
            }

            if (IsInfiniteCollider(collider))
            {
                if (proxy.InTree)
                {
                    m_Tree.Remove(proxy.Proxy);
                }

                proxy.Proxy = spatial::SpatialInvalidIndex;
                proxy.InTree = false;
                proxy.Infinite = true;
                AddInfinite(collider.ID);
                return;
            }

            const spatial::SpatialAABB bounds =
                WorldAABB(bodies.at(collider.Body), collider);

            const std::uint32_t categoryMask =
                BroadphaseCategoryMask(collider);

            if (proxy.InTree && proxy.CategoryMask == categoryMask)
            {
                [[maybe_unused]] const bool reinserted =
                    m_Tree.Update(proxy.Proxy, bounds);
            }
            else
            {
                if (proxy.InTree)
                {
                    m_Tree.Remove(proxy.Proxy);
                }

                proxy.Proxy =
                    m_Tree.Insert(
                        collider.ID,
                        bounds,
                        categoryMask);
                proxy.InTree = true;
            }

            proxy.CategoryMask = categoryMask;
            proxy.Infinite = false;
            RemoveInfinite(collider.ID);
        }

        void RemoveCollider(
            ColliderID collider)
        {
            if (collider >= m_Proxies.size())
            {
                return;
            }

            BroadphaseProxy& proxy =
                m_Proxies.at(collider);

            if (proxy.InTree)
            {
                m_Tree.Remove(proxy.Proxy);
            }

            proxy = {};
            RemoveInfinite(collider);
        }

        void Sync(
            const std::vector<RigidBody>& bodies,
            const std::vector<Collider>& colliders)
        {
            for (const Collider& collider : colliders)
            {
                AddOrUpdateCollider(bodies, collider);
            }
        }

        [[nodiscard]]
        std::vector<BroadphasePair> ComputePairs(
            const std::vector<RigidBody>& bodies,
            const std::vector<Collider>& colliders) const
        {
            std::vector<BroadphasePair> pairs;

            for (const spatial::SpatialPair& pair : m_Tree.ComputePairs())
            {
                if (ShouldPair(pair.A, pair.B, bodies, colliders))
                {
                    pairs.push_back(OrderedBroadphasePair(pair.A, pair.B));
                }
            }

            for (const Collider& collider : colliders)
            {
                if (collider.ID >= m_Proxies.size() ||
                    !m_Proxies.at(collider.ID).InTree)
                {
                    continue;
                }

                for (ColliderID infinite : m_InfiniteColliders)
                {
                    if (ShouldPair(collider.ID, infinite, bodies, colliders))
                    {
                        pairs.push_back(OrderedBroadphasePair(collider.ID, infinite));
                    }
                }
            }

            SortUnique(pairs);
            return pairs;
        }

    private:
        spatial::DynamicAABBTree m_Tree;
        std::vector<BroadphaseProxy> m_Proxies;
        std::vector<ColliderID> m_InfiniteColliders;

        void EnsureProxy(
            ColliderID collider)
        {
            if (collider >= m_Proxies.size())
            {
                m_Proxies.resize(static_cast<std::size_t>(collider) + 1u);
            }
        }

        void AddInfinite(
            ColliderID collider)
        {
            if (std::find(m_InfiniteColliders.begin(), m_InfiniteColliders.end(), collider) == m_InfiniteColliders.end())
            {
                m_InfiniteColliders.push_back(collider);
            }
        }

        void RemoveInfinite(
            ColliderID collider)
        {
            m_InfiniteColliders.erase(
                std::remove(m_InfiniteColliders.begin(), m_InfiniteColliders.end(), collider),
                m_InfiniteColliders.end());
        }

        [[nodiscard]]
        static bool ShouldPair(
            ColliderID a,
            ColliderID b,
            const std::vector<RigidBody>& bodies,
            const std::vector<Collider>& colliders)
        {
            if (a >= colliders.size() || b >= colliders.size())
            {
                return false;
            }

            const Collider& ca = colliders.at(a);
            const Collider& cb = colliders.at(b);
            return IsActiveCollider(ca) &&
                IsActiveCollider(cb) &&
                ca.Body < bodies.size() &&
                cb.Body < bodies.size() &&
                IsActiveBody(bodies.at(ca.Body)) &&
                IsActiveBody(bodies.at(cb.Body)) &&
                ca.Body != cb.Body &&
                CollisionFiltersAllow(ca, cb);
        }

        static void SortUnique(
            std::vector<BroadphasePair>& pairs)
        {
            std::sort(
                pairs.begin(),
                pairs.end(),
                [](const BroadphasePair& lhs, const BroadphasePair& rhs)
                {
                    return lhs.A == rhs.A ? lhs.B < rhs.B : lhs.A < rhs.A;
                });

            pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
        }
    };

    /// Input: bodies and colliders.
    /// Output: deterministic potentially-overlapping collider pairs.
    /// Task: reuse KairoSpatial's dynamic AABB tree for finite collider overlap
    /// queries while pairing infinite planes against all finite colliders.
    [[nodiscard]]
    inline std::vector<BroadphasePair> ComputeBroadphasePairs(
        const std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders)
    {
        BroadphaseWorld broadphase;
        broadphase.Sync(bodies, colliders);
        return broadphase.ComputePairs(bodies, colliders);
    }
}
