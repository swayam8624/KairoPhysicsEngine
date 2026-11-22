// TEACHING MAP: BroadphaseWorld is a *cache* of spatial proxies, not the
// authoritative owner of RigidBody or Collider. PhysicsWorld calls Sync before
// ComputePairs; finite colliders live in KairoSpatial::DynamicAABBTree while
// infinite colliders (for example, planes) use a separate ID list.
//
// The dynamic tree returns conservative possible overlaps, not shape contacts.
// Narrowphase must test geometry, ContactSolver responds to confirmed contacts,
// and PhysicsWorld applies policy overrides, events, and solver scheduling.
// SpatialLayerMask/CollisionFiltersAllow reduce candidate work but do not
// replace PhysicsWorld's richer collision-response rules and callbacks.
//
// C++23: `export module` names the interface; imports pull in Math,
// PhysicsMath, Spatial and collider contracts. See KairoSpatial's
// DynamicAABBTree.cppm and this repo's PhysicsWorld.cppm/Collider.cppm.
module;

#include <algorithm>
#include <cstdint>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Broadphase;

import Kairo.Foundation.Spatial;
import Kairo.Foundation.Math.Vector;
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
// Canonicalize pair identity for stable deduplication and event ordering.
    inline BroadphasePair OrderedBroadphasePair(
        ColliderID a,
        ColliderID b) noexcept
    {
        return a < b
            ? BroadphasePair{ a, b }
            : BroadphasePair{ b, a };
    }

    // This record describes an *index handle*, keyed in m_Proxies by
    // ColliderID. It does not own collider geometry or body transforms.
    // An infinite collider deliberately has no valid tree proxy.
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
        /// TEACHING METHOD: Rebuilds this spatial cache from empty state; PhysicsWorld owns the actual collider data.
        // Reinitializing the tree invalidates every SpatialIndex proxy.
        // Clear the parallel ID->proxy table and infinite list together;
        // retaining a previous handle after Clear would index a new tree.
        void Clear()
        {
            m_Tree = spatial::DynamicAABBTree{};
            m_Proxies.clear();
            m_InfiniteColliders.clear();
        }

        /// TEACHING METHOD: Creates or updates finite DynamicAABBTree proxies and handles infinite colliders separately; inactive collider state removes proxies.

        void AddOrUpdateCollider(
            const std::vector<RigidBody>& bodies,
            const Collider& collider)
        {
            // PhysicsWorld's ColliderID is a stable array-slot identity;
            // the tree's SpatialIndex is only a transient cache handle.
            // Make room for the ID before any lookup, even for disabled
            // colliders, so subsequent removal is idempotent.
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

            const std::uint32_t categoryMask =
                BroadphaseCategoryMask(collider);

            // A plane has no finite, conservative AABB. Move it out of
            // the tree *before* recording its infinite-list identity so
            // finite-vs-infinite queries still see it exactly once.
            if (IsInfiniteCollider(collider))
            {
                if (proxy.InTree)
                {
                    m_Tree.Remove(proxy.Proxy);
                }

                proxy.Proxy = spatial::SpatialInvalidIndex;
                proxy.InTree = false;
                proxy.Infinite = true;
                proxy.CategoryMask = categoryMask;
                AddInfinite(collider.ID);
                return;
            }

            const spatial::SpatialAABB bounds =
                WorldAABB(bodies.at(collider.Body), collider);

            // Spatial tree Update may retain a fat AABB if the new tight
            // bounds remain inside it. A changed category mask cannot
            // reuse that node metadata, so remove/reinsert instead.
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

        /// TEACHING METHOD: Removes indexing metadata and tree proxy for a ColliderID without deleting the physics collider itself.

        // Remove only indexing state. Collider and body arrays remain
        // owned by PhysicsWorld; a removed/disabled collider may occupy
        // its old ID slot in that authoritative array.
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

        /// TEACHING METHOD: Reconciles collider proxies against current PhysicsWorld body/collider arrays before candidate queries.

        // O(number of collider records) reconciliation. This expects
        // PhysicsWorld's array to retain inactive ID slots; it does not
        // independently discover IDs erased from a shrunken array.
        // Query helpers must call/ensure Sync after transform changes.
        void Sync(
            const std::vector<RigidBody>& bodies,
            const std::vector<Collider>& colliders)
        {
            for (const Collider& collider : colliders)
            {
                AddOrUpdateCollider(bodies, collider);
            }
        }

        /// TEACHING METHOD: Produces deduplicated candidates from dynamic-tree overlap and finite/infinite combinations. Narrowphase still tests contact geometry.

        [[nodiscard]]
        std::vector<BroadphasePair> ComputePairs(
            const std::vector<RigidBody>& bodies,
            const std::vector<Collider>& colliders) const
        {
            std::vector<BroadphasePair> pairs;

            // DynamicAABBTree owns finite-volume candidate discovery.
            // ShouldPair applies activity, same-body and category/mask rules;
            // the engine's response overrides are applied later.
            // Tree pairs are candidate AABB overlaps only. ShouldPair
            // rejects self-body, inactive, and category/mask-incompatible
            // pairs before the exact shape-dependent Narrowphase call.
            for (const spatial::SpatialPair& pair : m_Tree.ComputePairs())
            {
                if (ShouldPair(pair.A, pair.B, bodies, colliders))
                {
                    pairs.push_back(OrderedBroadphasePair(pair.A, pair.B));
                }
            }

            // Infinite planes cannot be represented by a finite AABB.
            // Pair every indexed finite collider with the unbounded list,
            // then canonicalize and deduplicate IDs below.
            // For F finite proxies and I infinite planes, the extra
            // pairing work is O(F*I) before filters. Infinite-vs-infinite
            // pairs are deliberately not generated in this path.
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

            // Canonical (minID,maxID) ordering allows deterministic
            // deduplication across spatial tree and infinite pair paths.
            // Narrowphase contact order can then rely on stable pairs.
            SortUnique(pairs);
            return pairs;
        }

        /// Input: finite world-space bounds and a caller layer mask.
        /// Output: candidate finite collider IDs from the dynamic AABB tree.
        /// Task: keep PhysicsWorld overlap queries proportional to spatial
        /// locality. The returned IDs are conservative candidates; callers
        /// must still test the actual collider shape when that distinction
        /// matters.
        /// TEACHING METHOD: Returns box-query candidates with layer filtering; not a proof of contact.
        [[nodiscard]]
        std::vector<ColliderID> QueryAABB(
            const spatial::SpatialAABB& query,
            std::uint32_t layerMask) const
        {
            return m_Tree.QueryAABB(query, layerMask).IDs;
        }

        /// Input: sphere center/radius and a caller layer mask.
        /// Output: finite collider IDs whose broadphase bounds overlap it.
        /// Task: provide the query primitive needed by radial gameplay effects,
        /// sensors, and future particle/cloth collision systems without a
        /// linear scan through every collider.
        /// TEACHING METHOD: Returns sphere-query candidates; callers must perform exact shape tests.
        [[nodiscard]]
        std::vector<ColliderID> QuerySphere(
            const Vec3f& center,
            float radius,
            std::uint32_t layerMask) const
        {
            return m_Tree.QuerySphere(center, radius, layerMask).IDs;
        }

        /// Input: conservative finite bounds and a caller layer mask.
        /// Output: tree candidates plus infinite colliders whose category matches.
        /// Task: serve swept-volume queries. Infinite planes must be included
        /// explicitly because they intentionally have no finite tree proxy.
        /// TEACHING METHOD: Combines finite-tree and infinite-body candidates for broader scene queries.
        [[nodiscard]]
        // Deliberately differs from QueryAABB(): the latter only asks
        // the finite tree, whereas this helper also returns infinite IDs
        // for subsequent shape-specific continuous/overlap queries.
        // Used by swept-volume/scene queries that must see *planes*
        // as well as finite tree hits. QueryAABB intentionally omits
        // planes; it is only a finite-tree query.
        std::vector<ColliderID> QueryAABBCandidates(
            const spatial::SpatialAABB& query,
            std::uint32_t layerMask) const
        {
            std::vector<ColliderID> candidates =
                m_Tree.QueryAABB(query, layerMask).IDs;

            for (const ColliderID collider : m_InfiniteColliders)
            {
                if (collider < m_Proxies.size() &&
                    (m_Proxies.at(collider).CategoryMask & layerMask) != 0u)
                {
                    candidates.push_back(collider);
                }
            }

            std::sort(candidates.begin(), candidates.end());
            candidates.erase(
                std::unique(candidates.begin(), candidates.end()),
                candidates.end());
            return candidates;
        }

        /// Input: normalized ray, finite/positive maximum distance, and layer mask.
        /// Output: all finite-tree candidates plus every active infinite collider.
        /// Task: defer exact shape intersection to PhysicsWorld while avoiding a
        /// full collider scan for the common finite-body case. Planes remain
        /// explicit candidates because an infinite plane cannot enter an AABB tree.
        /// TEACHING METHOD: Identifies bodies that might meet a ray; the narrowphase/scene query calculates actual intersections.
        [[nodiscard]]
        // This only reduces possible ray intersections. PhysicsWorld's
        // RaycastCollider then solves each shape's actual hit distance;
        // callers must not treat unordered candidate IDs as sorted hits.
        std::vector<ColliderID> RaycastCandidates(
            const spatial::SpatialRay& ray,
            float maxDistance,
            std::uint32_t layerMask) const
        {
            std::vector<ColliderID> candidates =
                m_Tree.QueryRay(ray, maxDistance, layerMask).IDs;

            for (const ColliderID collider : m_InfiniteColliders)
            {
                if (collider < m_Proxies.size() &&
                    (m_Proxies.at(collider).CategoryMask & layerMask) != 0u)
                {
                    candidates.push_back(collider);
                }
            }

            std::sort(candidates.begin(), candidates.end());
            candidates.erase(
                std::unique(candidates.begin(), candidates.end()),
                candidates.end());
            return candidates;
        }

    private:
        spatial::DynamicAABBTree m_Tree;
        std::vector<BroadphaseProxy> m_Proxies;
        std::vector<ColliderID> m_InfiniteColliders;

        /// TEACHING METHOD: Expands or reserves proxy metadata so a ColliderID has stable index storage.

        void EnsureProxy(
            ColliderID collider)
        {
            if (collider >= m_Proxies.size())
            {
                m_Proxies.resize(static_cast<std::size_t>(collider) + 1u);
            }
        }

        /// TEACHING METHOD: Stores unbounded collision geometry outside the finite AABB tree.

        void AddInfinite(
            ColliderID collider)
        {
            if (std::find(m_InfiniteColliders.begin(), m_InfiniteColliders.end(), collider) == m_InfiniteColliders.end())
            {
                m_InfiniteColliders.push_back(collider);
            }
        }

        /// TEACHING METHOD: Removes the auxiliary infinite-collider reference without destroying authoritative collider data.

        void RemoveInfinite(
            ColliderID collider)
        {
            m_InfiniteColliders.erase(
                std::remove(m_InfiniteColliders.begin(), m_InfiniteColliders.end(), collider),
                m_InfiniteColliders.end());
        }

        /// TEACHING METHOD: Applies activity, body and collision-filter rules to suppress irrelevant candidate pairs.

        [[nodiscard]]
        // A passed filter is not a promise of contact, blocking response,
        // or solver participation. In particular, trigger/event policy and
        // collision-response overrides live above this cache in PhysicsWorld.
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
            // Both colliders must have live parent bodies, must not
            // belong to the *same* body, and must pass mutual filter
            // masks. Triggers and policy overrides are handled later.
            return IsActiveCollider(ca) &&
                IsActiveCollider(cb) &&
                ca.Body < bodies.size() &&
                cb.Body < bodies.size() &&
                IsActiveBody(bodies.at(ca.Body)) &&
                IsActiveBody(bodies.at(cb.Body)) &&
                ca.Body != cb.Body &&
                CollisionFiltersAllow(ca, cb);
        }

        /// TEACHING METHOD: Canonicalizes and deduplicates pairs to avoid repeated narrowphase evaluations.

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
    /// TEACHING METHOD: Public convenience for computing broadphase candidates from provided state.
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
