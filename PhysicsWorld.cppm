module;

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.World;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Spatial;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Types;
import Kairo.Foundation.PhysicsEngine.Material;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.Broadphase;
import Kairo.Foundation.PhysicsEngine.Narrowphase;
import Kairo.Foundation.PhysicsEngine.ContactSolver;
import Kairo.Foundation.PhysicsEngine.Debug;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;
    using namespace kairo::foundation::geometry;

    enum class PhysicsContactEventType : std::uint8_t
    {
        Begin,
        Stay,
        End
    };

    struct PhysicsContactEvent final
    {
        BodyID BodyA = InvalidBodyID;
        BodyID BodyB = InvalidBodyID;
        ColliderID ColliderA = InvalidColliderID;
        ColliderID ColliderB = InvalidColliderID;
        bool IsTrigger = false;
        CollisionResponse Response = CollisionResponse::Block;
        PhysicsContactEventType Type = PhysicsContactEventType::Begin;
    };

    /// Input: result record returned by `PhysicsWorld::Raycast` and `RaycastAll`.
    /// Output: body/collider ids plus geometric hit data in world coordinates.
    /// Task: keep physics rays useful for sensors, lasers, picking, visibility
    /// probes, and sandbox/debug drawing without depending on any renderer.
    struct PhysicsRayHit final
    {
        BodyID Body = InvalidBodyID;
        ColliderID Collider = InvalidColliderID;
        Vec3f Point = Vec3f::Zero();
        Vec3f Normal = Vec3f::UnitX();
        float Distance = 0.0f;
        bool IsTrigger = false;
    };

    /// Input: a moving sphere query issued against the current PhysicsWorld.
    /// Output: first time-of-impact in normalized [0, 1] sweep time and world
    /// contact data. `Distance` is measured along the supplied displacement.
    /// Task: provide the minimum continuous query needed by ballistic
    /// projectiles and a future capsule-controller implementation without
    /// pretending discrete rigid-body contacts prevent tunnelling.
    struct PhysicsSweepHit final
    {
        BodyID Body = InvalidBodyID;
        ColliderID Collider = InvalidColliderID;
        Vec3f Point = Vec3f::Zero();
        Vec3f Normal = Vec3f::UnitX();
        float Distance = 0.0f;
        float TimeOfImpact = 0.0f;
        bool IsTrigger = false;
    };

    /// Input: one completed `PhysicsWorld::Step`.
    /// Output: wall-clock timings for the major engine phases in milliseconds.
    /// Task: support benchmark CSV output without forcing sandbox tools to
    /// guess timings from outside the world. Values are diagnostic snapshots,
    /// not deterministic simulation state.
    struct PhysicsStepProfile final
    {
        double StepMs = 0.0;
        double BroadphaseMs = 0.0;
        double NarrowphaseMs = 0.0;
        double SolverMs = 0.0;
    };

    struct CollisionPairResponseRule final
    {
        ColliderID ColliderA = InvalidColliderID;
        ColliderID ColliderB = InvalidColliderID;
        CollisionResponse Response = CollisionResponse::Block;
    };

    struct CollisionLayerResponseRule final
    {
        std::uint32_t LayerA = CollisionLayer::Default;
        std::uint32_t LayerB = CollisionLayer::Default;
        CollisionResponse Response = CollisionResponse::Block;
    };

    class PhysicsWorld final
    {
    public:
        using CollisionFilterCallback =
            std::function<CollisionResponse(const Collider&, const Collider&)>;

        using ContactEventCallback =
            std::function<void(const PhysicsContactEvent&)>;

        PhysicsStepSettings Settings;
        Vec3f Gravity = DefaultGravity;

        [[nodiscard]]
        bool IsValidBody(
            BodyID body) const noexcept
        {
            return body < m_Bodies.size() &&
                IsActiveBody(m_Bodies[body]);
        }

        [[nodiscard]]
        bool IsValidCollider(
            ColliderID collider) const noexcept
        {
            return collider < m_Colliders.size() &&
                IsActiveCollider(m_Colliders[collider]) &&
                IsValidBody(m_Colliders[collider].Body);
        }

        [[nodiscard]]
        BodyID CreateRigidBody(
            const RigidBodyDesc& desc)
        {
            const BodyID id =
                static_cast<BodyID>(m_Bodies.size());

            m_Bodies.push_back(MakeRigidBody(id, desc));
            return id;
        }

        [[nodiscard]]
        ColliderID AddCollider(
            BodyID body,
            ColliderShape shape,
            PhysicsMaterial material = {},
            const Vec3f& localCenter = Vec3f::Zero(),
            const Quaternionf& localRotation = Quaternionf::Identity())
        {
            if (!IsValidBody(body))
            {
                throw std::out_of_range("AddCollider failed: body id does not exist or is inactive.");
            }

            const ColliderID id =
                static_cast<ColliderID>(m_Colliders.size());

            m_Colliders.push_back(
                MakeCollider(id, body, shape, material, localCenter, localRotation));

            m_Broadphase.AddOrUpdateCollider(m_Bodies, m_Colliders.back());
            return id;
        }

        /// Input: active collider id.
        /// Output: none.
        /// Task: remove the collider from simulation while preserving stable
        /// vector-index ids for debug logs, cached contacts, and user handles.
        void RemoveCollider(
            ColliderID collider)
        {
            if (collider >= m_Colliders.size())
            {
                throw std::out_of_range("RemoveCollider failed: collider id does not exist.");
            }

            Collider& record =
                m_Colliders.at(collider);

            if (!record.Active)
            {
                return;
            }

            record.Active = false;
            record.Body = InvalidBodyID;
            m_Broadphase.RemoveCollider(collider);
            RemoveCollisionRulesForCollider(collider);
            RemoveCachedPairsForCollider(collider);
            RemoveCachedContactsForCollider(collider);
        }

        /// Input: collider id plus category and collision masks.
        /// Output: collider filter updated and broadphase proxy refreshed.
        /// Task: implement commercial-style collision filtering where
        /// `BelongsTo` describes what the collider is and `CollidesWith`
        /// describes what categories it accepts.
        void SetCollisionFilter(
            ColliderID collider,
            std::uint32_t belongsTo,
            std::uint32_t collidesWith)
        {
            if (!IsValidCollider(collider))
            {
                throw std::out_of_range("SetCollisionFilter failed: collider id does not exist or is inactive.");
            }

            Collider& record =
                m_Colliders.at(collider);

            record.BelongsTo = belongsTo;
            record.CollidesWith = collidesWith;
            record.LayerMask = belongsTo;
            m_Broadphase.AddOrUpdateCollider(m_Bodies, record);
        }

        /// Input: collider id and one or more layer bits.
        /// Output: collider category updated while keeping its existing mask.
        /// Task: provide the common engine operation "make this object type X"
        /// without forcing callers to restate all collision-mask bits.
        void SetColliderCollisionLayer(
            ColliderID collider,
            std::uint32_t belongsTo)
        {
            if (!IsValidCollider(collider))
            {
                throw std::out_of_range("SetColliderCollisionLayer failed: collider id does not exist or is inactive.");
            }

            Collider& record =
                m_Colliders.at(collider);

            record.BelongsTo = belongsTo;
            record.LayerMask = belongsTo;
            m_Broadphase.AddOrUpdateCollider(m_Bodies, record);
        }

        /// Input: collider id and accepted layer bits.
        /// Output: collider broadphase mask updated.
        /// Task: expose the common engine operation "this object is willing to
        /// consider these other object types" independently from its own layer.
        void SetColliderCollisionMask(
            ColliderID collider,
            std::uint32_t collidesWith)
        {
            if (!IsValidCollider(collider))
            {
                throw std::out_of_range("SetColliderCollisionMask failed: collider id does not exist or is inactive.");
            }

            m_Colliders.at(collider).CollidesWith = collidesWith;
        }

        /// Input: two layer masks and the response wanted between them.
        /// Output: world-level response rule installed or replaced.
        /// Task: emulate commercial engine channel tables. Broadphase masks
        /// still decide coarse visibility; this table decides whether an exact
        /// overlap becomes ignored, trigger-only, or blocking contact.
        void SetCollisionLayerResponse(
            std::uint32_t layerA,
            std::uint32_t layerB,
            CollisionResponse response)
        {
            ValidateLayerMask(layerA, "layerA");
            ValidateLayerMask(layerB, "layerB");
            ValidateCollisionResponse(response);

            const CollisionLayerResponseRule key =
                OrderedLayerResponseRule(layerA, layerB, response);

            const auto found =
                std::find_if(
                    m_LayerResponses.begin(),
                    m_LayerResponses.end(),
                    [&key](const CollisionLayerResponseRule& rule)
                    {
                        return SameLayerResponsePair(rule, key);
                    });

            if (found == m_LayerResponses.end())
            {
                m_LayerResponses.push_back(key);
            }
            else
            {
                found->Response = response;
            }
        }

        /// Input: two layer masks.
        /// Output: matching world-level response rule removed when present.
        /// Task: let tools/game code restore default mask/trigger behavior
        /// without recreating the world or every collider.
        void ClearCollisionLayerResponse(
            std::uint32_t layerA,
            std::uint32_t layerB)
        {
            ValidateLayerMask(layerA, "layerA");
            ValidateLayerMask(layerB, "layerB");

            const CollisionLayerResponseRule key =
                OrderedLayerResponseRule(layerA, layerB, CollisionResponse::Block);

            m_LayerResponses.erase(
                std::remove_if(
                    m_LayerResponses.begin(),
                    m_LayerResponses.end(),
                    [&key](const CollisionLayerResponseRule& rule)
                    {
                        return SameLayerResponsePair(rule, key);
                    }),
                m_LayerResponses.end());
        }

        /// Input: two active colliders and the exact response wanted for them.
        /// Output: pair-specific rule installed or replaced.
        /// Task: support high-priority gameplay exceptions such as "this
        /// projectile ignores the actor that spawned it" or "these two sensors
        /// overlap but do not block" without changing whole layers.
        void SetCollisionPairResponse(
            ColliderID colliderA,
            ColliderID colliderB,
            CollisionResponse response)
        {
            RequireColliderPair(colliderA, colliderB, "SetCollisionPairResponse");
            ValidateCollisionResponse(response);

            const CollisionPairResponseRule key =
                OrderedPairResponseRule(colliderA, colliderB, response);

            const auto found =
                std::find_if(
                    m_PairResponses.begin(),
                    m_PairResponses.end(),
                    [&key](const CollisionPairResponseRule& rule)
                    {
                        return SamePairResponsePair(rule, key);
                    });

            if (found == m_PairResponses.end())
            {
                m_PairResponses.push_back(key);
            }
            else
            {
                found->Response = response;
            }
        }

        /// Input: two collider ids.
        /// Output: exact pair response override removed when present.
        /// Task: restore layer/default collision behavior after a temporary
        /// gameplay exception expires.
        void ClearCollisionPairResponse(
            ColliderID colliderA,
            ColliderID colliderB)
        {
            RequireColliderPair(colliderA, colliderB, "ClearCollisionPairResponse");

            const CollisionPairResponseRule key =
                OrderedPairResponseRule(colliderA, colliderB, CollisionResponse::Block);

            m_PairResponses.erase(
                std::remove_if(
                    m_PairResponses.begin(),
                    m_PairResponses.end(),
                    [&key](const CollisionPairResponseRule& rule)
                    {
                        return SamePairResponsePair(rule, key);
                    }),
                m_PairResponses.end());
        }

        /// Input: optional callback returning a response for each exact contact.
        /// Output: callback stored and called during `Step` after narrowphase.
        /// Task: let gameplay/tooling make runtime decisions. Pair overrides
        /// still win first; this callback is the next hook before layer/default
        /// response. It should be deterministic and must not call `Step`.
        void SetCollisionFilterCallback(
            CollisionFilterCallback callback)
        {
            m_CollisionFilterCallback = std::move(callback);
        }

        /// Input: none.
        /// Output: runtime filter callback cleared.
        /// Task: restore pure data-driven collision response behavior.
        void ClearCollisionFilterCallback()
        {
            m_CollisionFilterCallback = nullptr;
        }

        /// Input: optional callback invoked for Begin/Stay/End events.
        /// Output: callback stored and invoked synchronously during `Step`.
        /// Task: provide game-engine style event hooks while keeping
        /// `ContactEvents()` available for polling and deterministic tests.
        void SetContactEventCallback(
            ContactEventCallback callback)
        {
            m_ContactEventCallback = std::move(callback);
        }

        /// Input: none.
        /// Output: contact event callback cleared.
        /// Task: stop synchronous event dispatch without changing simulation.
        void ClearContactEventCallback()
        {
            m_ContactEventCallback = nullptr;
        }

        /// Input: collider id and trigger state.
        /// Output: collider marked as solid or sensor.
        /// Task: triggers still produce contact manifolds for gameplay/events,
        /// but the solver skips their impulses and position correction.
        void SetColliderTrigger(
            ColliderID collider,
            bool isTrigger)
        {
            if (!IsValidCollider(collider))
            {
                throw std::out_of_range("SetColliderTrigger failed: collider id does not exist or is inactive.");
            }

            m_Colliders.at(collider).IsTrigger = isTrigger;
        }

        /// Input: active body id.
        /// Output: none.
        /// Task: deactivate a body and every collider attached to it without
        /// compacting storage. This gives users deletion-safe ids: an old id can
        /// be tested with `IsValidBody` and never aliases a future body.
        void DestroyRigidBody(
            BodyID body)
        {
            if (body >= m_Bodies.size())
            {
                throw std::out_of_range("DestroyRigidBody failed: body id does not exist.");
            }

            RigidBody& record =
                m_Bodies.at(body);

            if (!record.Active)
            {
                return;
            }

            for (Collider& collider : m_Colliders)
            {
                if (collider.Active && collider.Body == body)
                {
                    RemoveCollider(collider.ID);
                }
            }

            record.Active = false;
            record.Sleeping = true;
            record.State.LinearVelocity = Vec3f::Zero();
            record.State.AngularVelocity = Vec3f::Zero();
            ClearForces(record.Forces);
            RemoveCachedContactsForBody(body);
        }

        /// Input: active body id.
        /// Output: body marked awake for the next integration pass.
        /// Task: expose an explicit wake hook for gameplay code, tools, and
        /// tests without requiring direct mutation of the body record.
        void WakeBody(
            BodyID body)
        {
            if (!IsValidBody(body))
            {
                throw std::out_of_range("WakeBody failed: body id does not exist or is inactive.");
            }

            WakeRigidBody(m_Bodies.at(body));
        }

        void AddBodyForce(
            BodyID body,
            const Vec3f& force)
        {
            RigidBody& record =
                RequireMutableBody(body, "AddBodyForce");

            WakeRigidBody(record);
            kairo::foundation::physics::AddForce(record.Forces, force);
        }

        void AddBodyTorque(
            BodyID body,
            const Vec3f& torque)
        {
            RigidBody& record =
                RequireMutableBody(body, "AddBodyTorque");

            WakeRigidBody(record);
            kairo::foundation::physics::AddTorque(record.Forces, torque);
        }

        void AddBodyForceAtPoint(
            BodyID body,
            const Vec3f& force,
            const Vec3f& worldPoint)
        {
            RigidBody& record =
                RequireMutableBody(body, "AddBodyForceAtPoint");

            WakeRigidBody(record);
            kairo::foundation::physics::AddForceAtPoint(
                record.Forces,
                force,
                worldPoint,
                record.State.Position);
        }

        void ApplyBodyImpulseAtPoint(
            BodyID body,
            const Vec3f& impulse,
            const Vec3f& worldPoint)
        {
            RigidBody& record =
                RequireMutableBody(body, "ApplyBodyImpulseAtPoint");

            if (!IsDynamicBodyType(record))
            {
                return;
            }

            WakeRigidBody(record);
            ApplyImpulseAtPoint(
                record.State,
                record.Mass.InverseMass,
                WorldInverseInertia(record),
                impulse,
                worldPoint);
        }

        void Step(
            float dt)
        {
            const auto stepStart =
                std::chrono::steady_clock::now();

            ValidateStepSettings(Settings, dt);
            RequireFinite(Gravity, "Gravity");

            IntegrateForces(dt);
            IntegrateKinematicVelocities(dt);

            const auto broadphaseStart =
                std::chrono::steady_clock::now();

            m_Broadphase.Sync(m_Bodies, m_Colliders);
            m_LastPairs =
                m_Broadphase.ComputePairs(m_Bodies, m_Colliders);

            const auto broadphaseEnd =
                std::chrono::steady_clock::now();

            const auto narrowphaseStart =
                broadphaseEnd;

            m_LastContacts =
                ComputeContacts(m_Bodies, m_Colliders, m_LastPairs);

            ApplyCollisionResponses();
            UpdateContactEvents();
            DispatchContactEvents();
            WakeContactBodies();

            const auto narrowphaseEnd =
                std::chrono::steady_clock::now();

            const auto solverStart =
                narrowphaseEnd;

            RestoreContactCache();
            WarmStartContacts(m_Bodies, m_Colliders, m_LastContacts);

            SolveContacts(
                m_Bodies,
                m_Colliders,
                m_LastContacts,
                Settings,
                dt);

            StoreContactCache();

            CorrectPositions(
                m_Bodies,
                m_LastContacts,
                Settings);

            const auto solverEnd =
                std::chrono::steady_clock::now();

            IntegrateDynamicVelocities(dt);
            UpdateSleeping(dt);
            ClearForceAccumulators();

            const auto stepEnd =
                std::chrono::steady_clock::now();

            m_LastProfile =
            {
                ElapsedMilliseconds(stepStart, stepEnd),
                ElapsedMilliseconds(broadphaseStart, broadphaseEnd),
                ElapsedMilliseconds(narrowphaseStart, narrowphaseEnd),
                ElapsedMilliseconds(solverStart, solverEnd)
            };
        }

        /// Input: variable elapsed time, fixed simulation dt, and substep cap.
        /// Output: number of fixed `Step` calls executed.
        /// Task: let applications feed variable frame times while the physics
        /// simulation advances in deterministic fixed-size increments.
        std::uint32_t StepFixed(
            float elapsedTime,
            float fixedDt,
            std::uint32_t maxSubSteps = 8)
        {
            RequireNonNegative(elapsedTime, "elapsedTime");
            RequirePositive(fixedDt, "fixedDt");

            if (maxSubSteps == 0)
            {
                throw std::invalid_argument("maxSubSteps must be greater than zero.");
            }

            m_FixedAccumulator += elapsedTime;

            std::uint32_t steps = 0;
            while (m_FixedAccumulator >= fixedDt && steps < maxSubSteps)
            {
                Step(fixedDt);
                m_FixedAccumulator -= fixedDt;
                ++steps;
            }

            if (steps == maxSubSteps && m_FixedAccumulator >= fixedDt)
            {
                m_FixedAccumulator = 0.0f;
            }

            return steps;
        }

        [[nodiscard]]
        float FixedAccumulator() const noexcept
        {
            return m_FixedAccumulator;
        }

        void ResetFixedAccumulator() noexcept
        {
            m_FixedAccumulator = 0.0f;
        }

        [[nodiscard]]
        const std::vector<RigidBody>& Bodies() const noexcept
        {
            return m_Bodies;
        }

        [[nodiscard]]
        std::vector<RigidBody>& Bodies() noexcept
        {
            return m_Bodies;
        }

        [[nodiscard]]
        const std::vector<Collider>& Colliders() const noexcept
        {
            return m_Colliders;
        }

        [[nodiscard]]
        const std::vector<ContactManifold>& Contacts() const noexcept
        {
            return m_LastContacts;
        }

        [[nodiscard]]
        const std::vector<PhysicsContactEvent>& ContactEvents() const noexcept
        {
            return m_LastEvents;
        }

        [[nodiscard]]
        const std::vector<BroadphasePair>& BroadphasePairs() const noexcept
        {
            return m_LastPairs;
        }

        [[nodiscard]]
        const PhysicsStepProfile& LastStepProfile() const noexcept
        {
            return m_LastProfile;
        }

        [[nodiscard]]
        std::vector<DebugAABB> DebugAABBs() const
        {
            return CollectDebugAABBs(m_Bodies, m_Colliders);
        }

        [[nodiscard]]
        std::vector<DebugContact> DebugContacts() const
        {
            return CollectDebugContacts(m_LastContacts);
        }

        /// Input: none.
        /// Output: immutable world-space descriptions for every active collider.
        /// Task: publish renderer-ready debug geometry while preserving the
        /// PhysicsWorld ownership boundary and avoiding renderer dependencies.
        [[nodiscard]]
        std::vector<DebugShape> DebugShapes() const
        {
            return CollectDebugShapes(m_Bodies, m_Colliders);
        }

        [[nodiscard]]
        std::vector<ColliderID> QueryAABB(
            const AABBf& query,
            std::uint32_t layerMask = 0xFFFF'FFFFu) const
        {
            SyncBroadphaseForQueries();
            std::vector<ColliderID> result;
            for (const ColliderID candidate : m_Broadphase.QueryAABB(query, layerMask))
            {
                if (!IsValidCollider(candidate))
                {
                    continue;
                }

                const Collider& collider = m_Colliders.at(candidate);
                if (
                    IsInfiniteCollider(collider) ||
                    (BroadphaseCategoryMask(collider) & layerMask) == 0u)
                {
                    continue;
                }

                if (Intersects(WorldAABB(m_Bodies.at(collider.Body), collider), query))
                {
                    result.push_back(collider.ID);
                }
            }

            return result;
        }

        [[nodiscard]]
        std::vector<ColliderID> QuerySphere(
            const Vec3f& center,
            float radius,
            std::uint32_t layerMask = 0xFFFF'FFFFu) const
        {
            RequireFinite(center, "QuerySphere.center");
            RequirePositive(radius, "QuerySphere.radius");
            SyncBroadphaseForQueries();

            std::vector<ColliderID> result;
            const float radiusSq =
                radius * radius;

            for (const ColliderID candidate : m_Broadphase.QuerySphere(center, radius, layerMask))
            {
                if (!IsValidCollider(candidate))
                {
                    continue;
                }

                const Collider& collider = m_Colliders.at(candidate);
                if (
                    IsInfiniteCollider(collider) ||
                    (BroadphaseCategoryMask(collider) & layerMask) == 0u)
                {
                    continue;
                }

                if (WorldAABB(m_Bodies.at(collider.Body), collider).DistanceSquaredToPoint(center) <= radiusSq)
                {
                    result.push_back(collider.ID);
                }
            }

            return result;
        }

        /// Input: world-space ray origin, direction, optional max distance,
        /// optional layer mask, and optional collider id to ignore.
        /// Output: nearest ray hit, or `std::nullopt` when no collider is hit.
        /// Task: expose renderer-independent physics ray tests for selection,
        /// laser/sensor gameplay, visibility checks, and sandbox light-ray
        /// debugging. Direction is normalized internally; zero/NaN rays throw.
        [[nodiscard]]
        std::optional<PhysicsRayHit> Raycast(
            const Vec3f& origin,
            const Vec3f& direction,
            float maxDistance = std::numeric_limits<float>::infinity(),
            std::uint32_t layerMask = 0xFFFF'FFFFu,
            ColliderID ignoredCollider = InvalidColliderID) const
        {
            const std::vector<PhysicsRayHit> hits =
                RaycastAll(origin, direction, maxDistance, layerMask, ignoredCollider);

            if (hits.empty())
            {
                return std::nullopt;
            }

            return hits.front();
        }

        /// Input: world-space ray origin, direction, optional max distance,
        /// optional layer mask, and optional collider id to ignore.
        /// Output: every hit sorted by distance from nearest to farthest.
        /// Task: support multi-hit sensors and light-ray debugging where the
        /// caller wants to inspect all colliders along a segment, not only the
        /// first occluder.
        [[nodiscard]]
        std::vector<PhysicsRayHit> RaycastAll(
            const Vec3f& origin,
            const Vec3f& direction,
            float maxDistance = std::numeric_limits<float>::infinity(),
            std::uint32_t layerMask = 0xFFFF'FFFFu,
            ColliderID ignoredCollider = InvalidColliderID) const
        {
            RequireFinite(origin, "Raycast.origin");
            const Vec3f rayDirection =
                ValidateRayDirection(direction);
            const float rayMaxDistance =
                ValidateRayMaxDistance(maxDistance);
            SyncBroadphaseForQueries();

            std::vector<PhysicsRayHit> result;
            const spatial::SpatialRay ray{
                origin,
                rayDirection
            };
            for (const ColliderID candidate :
                m_Broadphase.RaycastCandidates(ray, rayMaxDistance, layerMask))
            {
                if (!IsValidCollider(candidate))
                {
                    continue;
                }

                const Collider& collider = m_Colliders.at(candidate);
                if (
                    collider.ID == ignoredCollider ||
                    (BroadphaseCategoryMask(collider) & layerMask) == 0u)
                {
                    continue;
                }

                PhysicsRayHit hit;
                if (RaycastCollider(
                    m_Bodies.at(collider.Body),
                    collider,
                    origin,
                    rayDirection,
                    rayMaxDistance,
                    hit))
                {
                    result.push_back(hit);
                }
            }

            std::sort(
                result.begin(),
                result.end(),
                [](const PhysicsRayHit& lhs, const PhysicsRayHit& rhs)
                {
                    if (lhs.Distance != rhs.Distance)
                    {
                        return lhs.Distance < rhs.Distance;
                    }

                    return lhs.Collider < rhs.Collider;
                });

            return result;
        }

        /// Input: start point, complete world-space displacement, positive
        /// moving-sphere radius, optional layer mask, and ignored collider.
        /// Output: nearest continuous sphere hit, or `std::nullopt`.
        /// Task: query a conservative swept AABB through the broadphase, then
        /// use a bounded conservative-advancement narrowphase against exact
        /// sphere, capsule, plane, AABB, and oriented-box distance functions.
        /// The method is deterministic and reports initial overlap at TOI zero.
        [[nodiscard]]
        std::optional<PhysicsSweepHit> SweepSphere(
            const Vec3f& start,
            const Vec3f& displacement,
            float radius,
            std::uint32_t layerMask = 0xFFFF'FFFFu,
            ColliderID ignoredCollider = InvalidColliderID) const
        {
            RequireFinite(start, "SweepSphere.start");
            RequireFinite(displacement, "SweepSphere.displacement");
            RequirePositive(radius, "SweepSphere.radius");

            const float travelDistance = displacement.Length();
            if (!std::isfinite(travelDistance) || travelDistance <= 1.0e-8f)
            {
                throw std::invalid_argument("SweepSphere.displacement must be non-zero.");
            }

            SyncBroadphaseForQueries();
            const Vec3f end = start + displacement;
            const Vec3f extent{ radius, radius, radius };
            const AABBf sweptBounds = AABBf::FromMinMax(
                Min(start, end) - extent,
                Max(start, end) + extent);

            std::optional<PhysicsSweepHit> closest;
            for (const ColliderID candidate :
                m_Broadphase.QueryAABBCandidates(sweptBounds, layerMask))
            {
                if (!IsValidCollider(candidate))
                {
                    continue;
                }

                const Collider& collider = m_Colliders.at(candidate);
                if (candidate == ignoredCollider ||
                    (BroadphaseCategoryMask(collider) & layerMask) == 0u)
                {
                    continue;
                }

                const auto hit = SweepSphereCollider(
                    m_Bodies.at(collider.Body), collider, start, displacement, radius);
                if (!hit || (closest && hit->Distance >= closest->Distance))
                {
                    continue;
                }

                closest = *hit;
            }

            return closest;
        }

    private:
        std::vector<RigidBody> m_Bodies;
        std::vector<Collider> m_Colliders;
        mutable BroadphaseWorld m_Broadphase;
        std::vector<BroadphasePair> m_LastPairs;
        std::vector<ContactManifold> m_LastContacts;
        std::vector<PhysicsContactEvent> m_LastEvents;
        float m_FixedAccumulator = 0.0f;
        PhysicsStepProfile m_LastProfile;
        std::vector<CollisionPairResponseRule> m_PairResponses;
        std::vector<CollisionLayerResponseRule> m_LayerResponses;
        CollisionFilterCallback m_CollisionFilterCallback;
        ContactEventCallback m_ContactEventCallback;

        struct ContactEventKey final
        {
            BodyID BodyA = InvalidBodyID;
            BodyID BodyB = InvalidBodyID;
            ColliderID ColliderA = InvalidColliderID;
            ColliderID ColliderB = InvalidColliderID;
            bool IsTrigger = false;
            CollisionResponse Response = CollisionResponse::Block;
        };

        std::vector<ContactEventKey> m_PreviousContactKeys;

        struct ContactCacheEntry final
        {
            BodyID BodyA = InvalidBodyID;
            BodyID BodyB = InvalidBodyID;
            ColliderID ColliderA = InvalidColliderID;
            ColliderID ColliderB = InvalidColliderID;
            std::uint32_t PointIndex = 0;
            float NormalImpulse = 0.0f;
            float TangentImpulse = 0.0f;
        };

        std::vector<ContactCacheEntry> m_ContactCache;

        /// Input: none; this is a logically-const maintenance operation.
        /// Output: the persistent dynamic tree reflects current body transforms.
        /// Task: callers may edit bodies through `Bodies()` between simulation
        /// steps. Query APIs must still remain correct, so they synchronize the
        /// broadphase before asking it for candidates rather than trusting a
        /// stale proxy from the last physics step.
        void SyncBroadphaseForQueries() const
        {
            m_Broadphase.Sync(m_Bodies, m_Colliders);
        }

        template<typename ClockTimePoint>
        [[nodiscard]]
        static double ElapsedMilliseconds(
            const ClockTimePoint& start,
            const ClockTimePoint& end)
        {
            return std::chrono::duration<double, std::milli>(end - start).count();
        }

        static void ValidateLayerMask(
            std::uint32_t layerMask,
            const char* name)
        {
            if (layerMask == 0u)
            {
                throw std::invalid_argument(std::string(name) + " must contain at least one collision layer bit.");
            }
        }

        static void ValidateCollisionResponse(
            CollisionResponse response)
        {
            switch (response)
            {
            case CollisionResponse::Ignore:
            case CollisionResponse::Trigger:
            case CollisionResponse::Block:
                return;
            }

            throw std::invalid_argument("CollisionResponse contains an invalid enum value.");
        }

        void RequireColliderPair(
            ColliderID colliderA,
            ColliderID colliderB,
            const char* operation) const
        {
            if (!IsValidCollider(colliderA) || !IsValidCollider(colliderB))
            {
                throw std::out_of_range(
                    std::string(operation) +
                    " failed: collider id does not exist or is inactive.");
            }

            if (colliderA == colliderB)
            {
                throw std::invalid_argument(
                    std::string(operation) +
                    " failed: collider pair must contain two different colliders.");
            }
        }

        [[nodiscard]]
        static CollisionPairResponseRule OrderedPairResponseRule(
            ColliderID colliderA,
            ColliderID colliderB,
            CollisionResponse response) noexcept
        {
            return colliderA < colliderB
                ? CollisionPairResponseRule{ colliderA, colliderB, response }
                : CollisionPairResponseRule{ colliderB, colliderA, response };
        }

        [[nodiscard]]
        static CollisionLayerResponseRule OrderedLayerResponseRule(
            std::uint32_t layerA,
            std::uint32_t layerB,
            CollisionResponse response) noexcept
        {
            return layerA < layerB
                ? CollisionLayerResponseRule{ layerA, layerB, response }
                : CollisionLayerResponseRule{ layerB, layerA, response };
        }

        [[nodiscard]]
        static bool SamePairResponsePair(
            const CollisionPairResponseRule& lhs,
            const CollisionPairResponseRule& rhs) noexcept
        {
            return lhs.ColliderA == rhs.ColliderA &&
                lhs.ColliderB == rhs.ColliderB;
        }

        [[nodiscard]]
        static bool SameLayerResponsePair(
            const CollisionLayerResponseRule& lhs,
            const CollisionLayerResponseRule& rhs) noexcept
        {
            return lhs.LayerA == rhs.LayerA &&
                lhs.LayerB == rhs.LayerB;
        }

        [[nodiscard]]
        static bool LayerRuleMatches(
            const CollisionLayerResponseRule& rule,
            const Collider& colliderA,
            const Collider& colliderB) noexcept
        {
            const std::uint32_t layerA =
                BroadphaseCategoryMask(colliderA);

            const std::uint32_t layerB =
                BroadphaseCategoryMask(colliderB);

            return ((layerA & rule.LayerA) != 0u && (layerB & rule.LayerB) != 0u) ||
                ((layerA & rule.LayerB) != 0u && (layerB & rule.LayerA) != 0u);
        }

        [[nodiscard]]
        std::optional<CollisionResponse> PairResponseOverride(
            ColliderID colliderA,
            ColliderID colliderB) const
        {
            const CollisionPairResponseRule key =
                OrderedPairResponseRule(colliderA, colliderB, CollisionResponse::Block);

            const auto found =
                std::find_if(
                    m_PairResponses.begin(),
                    m_PairResponses.end(),
                    [&key](const CollisionPairResponseRule& rule)
                    {
                        return SamePairResponsePair(rule, key);
                    });

            if (found == m_PairResponses.end())
            {
                return std::nullopt;
            }

            return found->Response;
        }

        [[nodiscard]]
        std::optional<CollisionResponse> LayerResponseOverride(
            const Collider& colliderA,
            const Collider& colliderB) const
        {
            const auto found =
                std::find_if(
                    m_LayerResponses.begin(),
                    m_LayerResponses.end(),
                    [&colliderA, &colliderB](const CollisionLayerResponseRule& rule)
                    {
                        return LayerRuleMatches(rule, colliderA, colliderB);
                    });

            if (found == m_LayerResponses.end())
            {
                return std::nullopt;
            }

            return found->Response;
        }

        [[nodiscard]]
        CollisionResponse ResolveCollisionResponse(
            const Collider& colliderA,
            const Collider& colliderB) const
        {
            if (const std::optional<CollisionResponse> pairOverride =
                PairResponseOverride(colliderA.ID, colliderB.ID))
            {
                ValidateCollisionResponse(*pairOverride);
                return *pairOverride;
            }

            if (m_CollisionFilterCallback)
            {
                const CollisionResponse callbackResponse =
                    m_CollisionFilterCallback(colliderA, colliderB);

                ValidateCollisionResponse(callbackResponse);
                return callbackResponse;
            }

            if (const std::optional<CollisionResponse> layerOverride =
                LayerResponseOverride(colliderA, colliderB))
            {
                ValidateCollisionResponse(*layerOverride);
                return *layerOverride;
            }

            return DefaultCollisionResponse(colliderA, colliderB);
        }

        void ApplyCollisionResponses()
        {
            std::vector<ContactManifold> filtered;
            filtered.reserve(m_LastContacts.size());

            for (ContactManifold manifold : m_LastContacts)
            {
                if (manifold.ColliderA >= m_Colliders.size() ||
                    manifold.ColliderB >= m_Colliders.size())
                {
                    continue;
                }

                const Collider& colliderA =
                    m_Colliders.at(manifold.ColliderA);

                const Collider& colliderB =
                    m_Colliders.at(manifold.ColliderB);

                const CollisionResponse response =
                    ResolveCollisionResponse(colliderA, colliderB);

                if (response == CollisionResponse::Ignore)
                {
                    continue;
                }

                manifold.IsTrigger =
                    response == CollisionResponse::Trigger;

                filtered.push_back(manifold);
            }

            m_LastContacts = std::move(filtered);
        }

        void DispatchContactEvents()
        {
            if (!m_ContactEventCallback)
            {
                return;
            }

            for (const PhysicsContactEvent& event : m_LastEvents)
            {
                m_ContactEventCallback(event);
            }
        }

        void RemoveCollisionRulesForCollider(
            ColliderID collider)
        {
            m_PairResponses.erase(
                std::remove_if(
                    m_PairResponses.begin(),
                    m_PairResponses.end(),
                    [collider](const CollisionPairResponseRule& rule)
                    {
                        return rule.ColliderA == collider ||
                            rule.ColliderB == collider;
                    }),
                m_PairResponses.end());
        }

        [[nodiscard]]
        static Vec3f ValidateRayDirection(
            const Vec3f& direction)
        {
            RequireFinite(direction, "Raycast.direction");

            const float lengthSq =
                direction.LengthSquared();

            if (lengthSq <= 1.0e-12f)
            {
                throw std::invalid_argument("Raycast.direction must be non-zero.");
            }

            return direction / std::sqrt(lengthSq);
        }

        [[nodiscard]]
        static float ValidateRayMaxDistance(
            float maxDistance)
        {
            if (std::isnan(maxDistance) || maxDistance <= 0.0f)
            {
                throw std::invalid_argument("Raycast.maxDistance must be greater than zero.");
            }

            return maxDistance;
        }

        [[nodiscard]]
        static Vec3f AxisVector(
            std::size_t axis,
            float sign) noexcept
        {
            if (axis == 0)
            {
                return Vec3f{ sign, 0.0f, 0.0f };
            }

            if (axis == 1)
            {
                return Vec3f{ 0.0f, sign, 0.0f };
            }

            return Vec3f{ 0.0f, 0.0f, sign };
        }

        [[nodiscard]]
        static bool RaycastAxisAlignedBox(
            const Vec3f& origin,
            const Vec3f& direction,
            const Vec3f& min,
            const Vec3f& max,
            float maxDistance,
            float& distance,
            Vec3f& normal)
        {
            float nearTime =
                0.0f;

            float farTime =
                maxDistance;

            std::size_t nearAxis =
                0;

            float nearSign =
                0.0f;

            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                const float rayOrigin =
                    origin[axis];

                const float rayDirection =
                    direction[axis];

                if (std::abs(rayDirection) <= 1.0e-8f)
                {
                    if (rayOrigin < min[axis] || rayOrigin > max[axis])
                    {
                        return false;
                    }

                    continue;
                }

                const float inverseDirection =
                    1.0f / rayDirection;

                float axisNear =
                    (min[axis] - rayOrigin) * inverseDirection;

                float axisFar =
                    (max[axis] - rayOrigin) * inverseDirection;

                float axisSign =
                    rayDirection > 0.0f ? -1.0f : 1.0f;

                if (axisNear > axisFar)
                {
                    std::swap(axisNear, axisFar);
                    axisSign = -axisSign;
                }

                if (axisNear > nearTime)
                {
                    nearTime = axisNear;
                    nearAxis = axis;
                    nearSign = axisSign;
                }

                farTime =
                    std::min(farTime, axisFar);

                if (nearTime > farTime)
                {
                    return false;
                }
            }

            if (nearTime > maxDistance)
            {
                return false;
            }

            distance =
                nearTime;

            normal =
                nearSign == 0.0f
                    ? -direction
                    : AxisVector(nearAxis, nearSign);

            return true;
        }

        [[nodiscard]]
        static bool RaycastSphere(
            const Vec3f& origin,
            const Vec3f& direction,
            const Vec3f& center,
            float radius,
            float maxDistance,
            float& distance,
            Vec3f& normal)
        {
            const Vec3f offset =
                origin - center;

            const float projected =
                Dot(offset, direction);

            const float radiusSq =
                radius * radius;

            const float centerDistanceSq =
                offset.LengthSquared();

            if (centerDistanceSq <= radiusSq)
            {
                distance = 0.0f;
                normal = SafeNormalize(offset, -direction);
                return true;
            }

            if (projected > 0.0f)
            {
                return false;
            }

            const float discriminant =
                projected * projected - (centerDistanceSq - radiusSq);

            if (discriminant < 0.0f)
            {
                return false;
            }

            distance =
                -projected - std::sqrt(discriminant);

            if (distance < 0.0f || distance > maxDistance)
            {
                return false;
            }

            const Vec3f point =
                origin + direction * distance;

            normal =
                SafeNormalize(point - center, -direction);

            return true;
        }

        [[nodiscard]]
        static bool RaycastCapsule(
            const Vec3f& origin,
            const Vec3f& direction,
            const CapsuleSegment& capsule,
            float maxDistance,
            float& distance,
            Vec3f& normal)
        {
            const Vec3f axis = capsule.B - capsule.A;
            const Vec3f originOffset = origin - capsule.A;
            const float axisLengthSq = axis.LengthSquared();
            if (axisLengthSq <= 1.0e-12f)
            {
                return RaycastSphere(
                    origin, direction, capsule.A, capsule.Radius,
                    maxDistance, distance, normal);
            }

            const float axisDirection = Dot(axis, direction);
            const float axisOrigin = Dot(axis, originOffset);
            const float rayOrigin = Dot(direction, originOffset);
            const float originLengthSq = originOffset.LengthSquared();
            const float quadraticA = axisLengthSq - axisDirection * axisDirection;
            const float quadraticB = axisLengthSq * rayOrigin - axisOrigin * axisDirection;
            const float quadraticC = axisLengthSq * originLengthSq - axisOrigin * axisOrigin -
                capsule.Radius * capsule.Radius * axisLengthSq;
            const float discriminant = quadraticB * quadraticB - quadraticA * quadraticC;

            if (std::abs(quadraticA) > 1.0e-10f && discriminant >= 0.0f)
            {
                const float candidate = (-quadraticB - std::sqrt(discriminant)) / quadraticA;
                const float axial = axisOrigin + candidate * axisDirection;
                if (candidate >= 0.0f && candidate <= maxDistance &&
                    axial >= 0.0f && axial <= axisLengthSq)
                {
                    distance = candidate;
                    const Vec3f point = origin + direction * distance;
                    const Vec3f center = capsule.A + axis * (axial / axisLengthSq);
                    normal = SafeNormalize(point - center, -direction);
                    return true;
                }
            }

            float firstDistance = std::numeric_limits<float>::infinity();
            float secondDistance = std::numeric_limits<float>::infinity();
            Vec3f firstNormal = -direction;
            Vec3f secondNormal = -direction;
            const bool firstHit = RaycastSphere(
                origin, direction, capsule.A, capsule.Radius,
                maxDistance, firstDistance, firstNormal);
            const bool secondHit = RaycastSphere(
                origin, direction, capsule.B, capsule.Radius,
                maxDistance, secondDistance, secondNormal);
            if (!firstHit && !secondHit)
            {
                return false;
            }

            if (firstHit && (!secondHit || firstDistance <= secondDistance))
            {
                distance = firstDistance;
                normal = firstNormal;
            }
            else
            {
                distance = secondDistance;
                normal = secondNormal;
            }
            return true;
        }

        [[nodiscard]]
        static bool RaycastPlane(
            const Vec3f& origin,
            const Vec3f& direction,
            const PlaneCollider& plane,
            float maxDistance,
            float& distance,
            Vec3f& normal)
        {
            const float denominator =
                Dot(plane.Normal, direction);

            if (std::abs(denominator) <= 1.0e-8f)
            {
                return false;
            }

            distance =
                -(Dot(plane.Normal, origin) + plane.Distance) / denominator;

            if (distance < 0.0f || distance > maxDistance)
            {
                return false;
            }

            normal =
                denominator < 0.0f ? plane.Normal : -plane.Normal;

            return true;
        }

        [[nodiscard]]
        static bool RaycastOrientedBox(
            const Vec3f& origin,
            const Vec3f& direction,
            const OrientedBoxFrame& frame,
            float maxDistance,
            float& distance,
            Vec3f& normal)
        {
            const Vec3f delta =
                origin - frame.Center;

            const Vec3f localOrigin
            {
                Dot(delta, frame.Axes[0]),
                Dot(delta, frame.Axes[1]),
                Dot(delta, frame.Axes[2])
            };

            const Vec3f localDirection
            {
                Dot(direction, frame.Axes[0]),
                Dot(direction, frame.Axes[1]),
                Dot(direction, frame.Axes[2])
            };

            Vec3f localNormal =
                Vec3f::Zero();

            if (!RaycastAxisAlignedBox(
                localOrigin,
                localDirection,
                -frame.HalfExtents,
                frame.HalfExtents,
                maxDistance,
                distance,
                localNormal))
            {
                return false;
            }

            normal =
                SafeNormalize(
                    frame.Axes[0] * localNormal.x +
                    frame.Axes[1] * localNormal.y +
                    frame.Axes[2] * localNormal.z,
                    -direction);

            return true;
        }

        [[nodiscard]]
        static bool RaycastCollider(
            const RigidBody& body,
            const Collider& collider,
            const Vec3f& origin,
            const Vec3f& direction,
            float maxDistance,
            PhysicsRayHit& hit)
        {
            float distance =
                0.0f;

            Vec3f normal =
                -direction;

            const Vec3f center =
                WorldColliderCenter(body, collider);

            bool intersects =
                false;

            if (const auto* sphere = std::get_if<SphereCollider>(&collider.Shape))
            {
                intersects =
                    RaycastSphere(
                        origin,
                        direction,
                        center,
                        sphere->Radius,
                        maxDistance,
                        distance,
                    normal);
            }
            else if (const auto* capsule = std::get_if<CapsuleCollider>(&collider.Shape))
            {
                intersects = RaycastCapsule(
                    origin,
                    direction,
                    WorldCapsuleSegment(body, collider, *capsule),
                    maxDistance,
                    distance,
                    normal);
            }
            else if (const auto* box = std::get_if<AABBCollider>(&collider.Shape))
            {
                intersects =
                    RaycastAxisAlignedBox(
                        origin,
                        direction,
                        center - box->HalfExtents,
                        center + box->HalfExtents,
                        maxDistance,
                        distance,
                        normal);
            }
            else if (const auto* box = std::get_if<BoxCollider>(&collider.Shape))
            {
                intersects =
                    RaycastOrientedBox(
                        origin,
                        direction,
                        WorldBoxFrame(body, collider, box->HalfExtents),
                        maxDistance,
                        distance,
                        normal);
            }
            else if (const auto* plane = std::get_if<PlaneCollider>(&collider.Shape))
            {
                intersects =
                    RaycastPlane(
                        origin,
                        direction,
                        *plane,
                        maxDistance,
                        distance,
                        normal);
            }

            if (!intersects)
            {
                return false;
            }

            hit =
            {
                body.ID,
                collider.ID,
                origin + direction * distance,
                normal,
                distance,
                collider.IsTrigger
            };

            return true;
        }

        [[nodiscard]]
        static float PointColliderSeparation(
            const RigidBody& body,
            const Collider& collider,
            const Vec3f& point,
            const Vec3f& fallbackNormal,
            Vec3f& normal)
        {
            const Vec3f center = WorldColliderCenter(body, collider);
            if (const auto* sphere = std::get_if<SphereCollider>(&collider.Shape))
            {
                const Vec3f delta = point - center;
                const float distance = delta.Length();
                normal = SafeNormalize(delta, fallbackNormal);
                return std::max(0.0f, distance - sphere->Radius);
            }

            if (const auto* capsule = std::get_if<CapsuleCollider>(&collider.Shape))
            {
                const CapsuleSegment segment = WorldCapsuleSegment(body, collider, *capsule);
                const Vec3f closest = ClosestPointOnSegment(segment.A, segment.B, point);
                const Vec3f delta = point - closest;
                const float distance = delta.Length();
                normal = SafeNormalize(delta, fallbackNormal);
                return std::max(0.0f, distance - capsule->Radius);
            }

            if (const auto* plane = std::get_if<PlaneCollider>(&collider.Shape))
            {
                const float signedDistance = Dot(plane->Normal, point) + plane->Distance;
                normal = signedDistance >= 0.0f ? plane->Normal : -plane->Normal;
                return std::abs(signedDistance);
            }

            OrientedBoxFrame box;
            if (const auto* aabb = std::get_if<AABBCollider>(&collider.Shape))
            {
                box = { center, { Vec3f::UnitX(), Vec3f::UnitY(), Vec3f::UnitZ() }, aabb->HalfExtents };
            }
            else if (const auto* oriented = std::get_if<BoxCollider>(&collider.Shape))
            {
                box = WorldBoxFrame(body, collider, oriented->HalfExtents);
            }
            else
            {
                normal = fallbackNormal;
                return std::numeric_limits<float>::infinity();
            }

            const Vec3f closest = ClosestPointOnBox(box, point);
            const Vec3f delta = point - closest;
            const float distance = delta.Length();
            normal = distance > 1.0e-6f
                ? delta / distance
                : SphereBoxFallbackNormal(box, point);
            return distance;
        }

        [[nodiscard]]
        static std::optional<PhysicsSweepHit> SweepSphereCollider(
            const RigidBody& body,
            const Collider& collider,
            const Vec3f& start,
            const Vec3f& displacement,
            float radius)
        {
            const float travelDistance = displacement.Length();
            const Vec3f direction = displacement / travelDistance;
            float time = 0.0f;
            constexpr float tolerance = 1.0e-4f;

            for (int iteration = 0; iteration < 48; ++iteration)
            {
                const Vec3f center = start + displacement * time;
                Vec3f normal = -direction;
                const float separation = PointColliderSeparation(
                    body, collider, center, -direction, normal) - radius;

                if (separation <= tolerance)
                {
                    return PhysicsSweepHit{
                        body.ID,
                        collider.ID,
                        center - normal * radius,
                        normal,
                        time * travelDistance,
                        time,
                        collider.IsTrigger
                    };
                }

                time += separation / travelDistance;
                if (time > 1.0f)
                {
                    return std::nullopt;
                }
            }

            return std::nullopt;
        }

        void IntegrateForces(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (body.Sleeping && HasAccumulatedForce(body))
                {
                    WakeRigidBody(body);
                }

                if (!IsDynamic(body))
                {
                    continue;
                }

                const Vec3f acceleration =
                    (body.EnableGravity ? Gravity * (Settings.GravityScale * body.GravityScale) : Vec3f::Zero()) +
                    body.Forces.Force * body.Mass.InverseMass;

                body.State.LinearVelocity += acceleration * dt;
                body.State.AngularVelocity +=
                    WorldInverseInertia(body) * body.Forces.Torque * dt;

                ApplyVelocityControls(body, dt);
            }
        }

        void IntegrateKinematicVelocities(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (!IsKinematic(body))
                {
                    continue;
                }

                ApplyVelocityControls(body, dt);
                AdvanceMotionState(body.State, dt);
            }
        }

        void IntegrateDynamicVelocities(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (!IsDynamic(body))
                {
                    continue;
                }

                ApplyVelocityControls(body, dt);
                AdvanceMotionState(body.State, dt);
            }
        }

        void ClearForceAccumulators() noexcept
        {
            for (RigidBody& body : m_Bodies)
            {
                ClearForces(body.Forces);
            }
        }

        RigidBody& RequireMutableBody(
            BodyID body,
            const char* operation)
        {
            if (!IsValidBody(body))
            {
                throw std::out_of_range(
                    std::string(operation) +
                    " failed: body id does not exist or is inactive.");
            }

            return m_Bodies.at(body);
        }

        [[nodiscard]]
        static CollisionResponse ContactResponse(
            const ContactManifold& manifold) noexcept
        {
            return manifold.IsTrigger
                ? CollisionResponse::Trigger
                : CollisionResponse::Block;
        }

        [[nodiscard]]
        static ContactEventKey MakeContactEventKey(
            const ContactManifold& manifold) noexcept
        {
            return
            {
                manifold.BodyA,
                manifold.BodyB,
                manifold.ColliderA,
                manifold.ColliderB,
                manifold.IsTrigger,
                ContactResponse(manifold)
            };
        }

        [[nodiscard]]
        static bool SameContactEventKey(
            const ContactEventKey& lhs,
            const ContactEventKey& rhs) noexcept
        {
            return lhs.BodyA == rhs.BodyA &&
                lhs.BodyB == rhs.BodyB &&
                lhs.ColliderA == rhs.ColliderA &&
                lhs.ColliderB == rhs.ColliderB &&
                lhs.IsTrigger == rhs.IsTrigger &&
                lhs.Response == rhs.Response;
        }

        [[nodiscard]]
        static bool LessContactEventKey(
            const ContactEventKey& lhs,
            const ContactEventKey& rhs) noexcept
        {
            if (lhs.BodyA != rhs.BodyA)
            {
                return lhs.BodyA < rhs.BodyA;
            }
            if (lhs.BodyB != rhs.BodyB)
            {
                return lhs.BodyB < rhs.BodyB;
            }
            if (lhs.ColliderA != rhs.ColliderA)
            {
                return lhs.ColliderA < rhs.ColliderA;
            }
            if (lhs.ColliderB != rhs.ColliderB)
            {
                return lhs.ColliderB < rhs.ColliderB;
            }
            if (lhs.IsTrigger != rhs.IsTrigger)
            {
                return lhs.IsTrigger < rhs.IsTrigger;
            }
            return static_cast<std::uint8_t>(lhs.Response) < static_cast<std::uint8_t>(rhs.Response);
        }

        [[nodiscard]]
        static PhysicsContactEvent MakeContactEvent(
            const ContactEventKey& key,
            PhysicsContactEventType type) noexcept
        {
            return
            {
                key.BodyA,
                key.BodyB,
                key.ColliderA,
                key.ColliderB,
                key.IsTrigger,
                key.Response,
                type
            };
        }

        void UpdateContactEvents()
        {
            std::vector<ContactEventKey> current;
            for (const ContactManifold& manifold : m_LastContacts)
            {
                current.push_back(MakeContactEventKey(manifold));
            }

            std::sort(current.begin(), current.end(), LessContactEventKey);
            current.erase(
                std::unique(
                    current.begin(),
                    current.end(),
                    SameContactEventKey),
                current.end());

            m_LastEvents.clear();

            for (const ContactEventKey& key : current)
            {
                const bool existed =
                    std::find_if(
                        m_PreviousContactKeys.begin(),
                        m_PreviousContactKeys.end(),
                        [&key](const ContactEventKey& previous)
                        {
                            return SameContactEventKey(key, previous);
                        }) != m_PreviousContactKeys.end();

                m_LastEvents.push_back(
                    MakeContactEvent(
                        key,
                        existed ? PhysicsContactEventType::Stay : PhysicsContactEventType::Begin));
            }

            for (const ContactEventKey& previous : m_PreviousContactKeys)
            {
                const bool stillExists =
                    std::find_if(
                        current.begin(),
                        current.end(),
                        [&previous](const ContactEventKey& key)
                        {
                            return SameContactEventKey(key, previous);
                        }) != current.end();

                if (!stillExists)
                {
                    m_LastEvents.push_back(
                        MakeContactEvent(previous, PhysicsContactEventType::End));
                }
            }

            std::sort(
                m_LastEvents.begin(),
                m_LastEvents.end(),
                [](const PhysicsContactEvent& lhs, const PhysicsContactEvent& rhs)
                {
                    return LessContactEventKey(
                        ContactEventKey{ lhs.BodyA, lhs.BodyB, lhs.ColliderA, lhs.ColliderB, lhs.IsTrigger, lhs.Response },
                        ContactEventKey{ rhs.BodyA, rhs.BodyB, rhs.ColliderA, rhs.ColliderB, rhs.IsTrigger, rhs.Response });
                });

            m_PreviousContactKeys = std::move(current);
        }

        static void AdvanceMotionState(
            MotionState& state,
            float dt)
        {
            state.Position += state.LinearVelocity * dt;

            if (state.AngularVelocity.LengthSquared() > 1.0e-12f)
            {
                state.Rotation =
                    IntegrateAngularVelocity(
                        state.Rotation,
                        state.AngularVelocity,
                        dt);
            }
        }

        static void ClampVelocity(
            Vec3f& velocity,
            float maxSpeed)
        {
            const float maxSpeedSq =
                maxSpeed * maxSpeed;

            const float lengthSq =
                velocity.LengthSquared();

            if (lengthSq > maxSpeedSq)
            {
                velocity =
                    velocity * (maxSpeed / std::sqrt(lengthSq));
            }
        }

        static void ApplyVelocityControls(
            RigidBody& body,
            float dt)
        {
            if (body.LinearDamping > 0.0f)
            {
                body.State.LinearVelocity *=
                    std::exp(-body.LinearDamping * dt);
            }

            if (body.AngularDamping > 0.0f)
            {
                body.State.AngularVelocity *=
                    std::exp(-body.AngularDamping * dt);
            }

            ClampVelocity(body.State.LinearVelocity, body.MaxLinearSpeed);
            ClampVelocity(body.State.AngularVelocity, body.MaxAngularSpeed);
        }

        void WakeContactBodies()
        {
            for (const ContactManifold& manifold : m_LastContacts)
            {
                if (manifold.IsTrigger)
                {
                    continue;
                }

                if (manifold.BodyA >= m_Bodies.size() ||
                    manifold.BodyB >= m_Bodies.size())
                {
                    continue;
                }

                RigidBody& bodyA =
                    m_Bodies.at(manifold.BodyA);

                RigidBody& bodyB =
                    m_Bodies.at(manifold.BodyB);

                const bool wakeA =
                    IsDynamicBodyType(bodyA) &&
                    bodyA.Sleeping &&
                    IsDynamicBodyType(bodyB) &&
                    !bodyB.Sleeping;

                const bool wakeB =
                    IsDynamicBodyType(bodyB) &&
                    bodyB.Sleeping &&
                    IsDynamicBodyType(bodyA) &&
                    !bodyA.Sleeping;

                if (wakeA)
                {
                    WakeRigidBody(bodyA);
                }

                if (wakeB)
                {
                    WakeRigidBody(bodyB);
                }
            }
        }

        void UpdateSleeping(
            float dt)
        {
            for (RigidBody& body : m_Bodies)
            {
                if (!IsDynamicBodyType(body))
                {
                    continue;
                }

                if (!Settings.EnableSleeping || !body.AllowSleeping)
                {
                    WakeRigidBody(body);
                    continue;
                }

                if (HasAccumulatedForce(body))
                {
                    WakeRigidBody(body);
                    continue;
                }

                const bool slowLinear =
                    body.State.LinearVelocity.LengthSquared() <=
                    Settings.SleepLinearSpeed * Settings.SleepLinearSpeed;

                const bool slowAngular =
                    body.State.AngularVelocity.LengthSquared() <=
                    Settings.SleepAngularSpeed * Settings.SleepAngularSpeed;

                if (slowLinear && slowAngular)
                {
                    body.SleepTimer += dt;
                    if (body.SleepTimer >= Settings.SleepTime)
                    {
                        body.Sleeping = true;
                        body.State.LinearVelocity = Vec3f::Zero();
                        body.State.AngularVelocity = Vec3f::Zero();
                    }
                }
                else
                {
                    WakeRigidBody(body);
                }
            }
        }

        void RemoveCachedPairsForCollider(
            ColliderID collider)
        {
            m_LastPairs.erase(
                std::remove_if(
                    m_LastPairs.begin(),
                    m_LastPairs.end(),
                    [collider](const BroadphasePair& pair)
                    {
                        return pair.A == collider || pair.B == collider;
                    }),
                m_LastPairs.end());
        }

        void RemoveCachedContactsForCollider(
            ColliderID collider)
        {
            m_LastContacts.erase(
                std::remove_if(
                    m_LastContacts.begin(),
                    m_LastContacts.end(),
                    [collider](const ContactManifold& manifold)
                    {
                        return manifold.ColliderA == collider ||
                            manifold.ColliderB == collider;
                    }),
                m_LastContacts.end());

            m_ContactCache.erase(
                std::remove_if(
                    m_ContactCache.begin(),
                    m_ContactCache.end(),
                    [collider](const ContactCacheEntry& entry)
                    {
                        return entry.ColliderA == collider ||
                            entry.ColliderB == collider;
                    }),
                m_ContactCache.end());

            m_PreviousContactKeys.erase(
                std::remove_if(
                    m_PreviousContactKeys.begin(),
                    m_PreviousContactKeys.end(),
                    [collider](const ContactEventKey& entry)
                    {
                        return entry.ColliderA == collider ||
                            entry.ColliderB == collider;
                    }),
                m_PreviousContactKeys.end());

            m_LastEvents.erase(
                std::remove_if(
                    m_LastEvents.begin(),
                    m_LastEvents.end(),
                    [collider](const PhysicsContactEvent& event)
                    {
                        return event.ColliderA == collider ||
                            event.ColliderB == collider;
                    }),
                m_LastEvents.end());
        }

        void RemoveCachedContactsForBody(
            BodyID body)
        {
            m_LastContacts.erase(
                std::remove_if(
                    m_LastContacts.begin(),
                    m_LastContacts.end(),
                    [body](const ContactManifold& manifold)
                    {
                        return manifold.BodyA == body ||
                            manifold.BodyB == body;
                    }),
                m_LastContacts.end());

            m_ContactCache.erase(
                std::remove_if(
                    m_ContactCache.begin(),
                    m_ContactCache.end(),
                    [body](const ContactCacheEntry& entry)
                    {
                        return entry.BodyA == body ||
                            entry.BodyB == body;
                    }),
                m_ContactCache.end());

            m_PreviousContactKeys.erase(
                std::remove_if(
                    m_PreviousContactKeys.begin(),
                    m_PreviousContactKeys.end(),
                    [body](const ContactEventKey& entry)
                    {
                        return entry.BodyA == body ||
                            entry.BodyB == body;
                    }),
                m_PreviousContactKeys.end());

            m_LastEvents.erase(
                std::remove_if(
                    m_LastEvents.begin(),
                    m_LastEvents.end(),
                    [body](const PhysicsContactEvent& event)
                    {
                        return event.BodyA == body ||
                            event.BodyB == body;
                    }),
                m_LastEvents.end());
        }

        [[nodiscard]]
        static bool SameCacheKey(
            const ContactCacheEntry& entry,
            const ContactManifold& manifold,
            std::uint32_t pointIndex) noexcept
        {
            return entry.BodyA == manifold.BodyA &&
                entry.BodyB == manifold.BodyB &&
                entry.ColliderA == manifold.ColliderA &&
                entry.ColliderB == manifold.ColliderB &&
                entry.PointIndex == pointIndex;
        }

        void RestoreContactCache()
        {
            for (ContactManifold& manifold : m_LastContacts)
            {
                for (std::size_t pointIndex = 0; pointIndex < manifold.Points.size(); ++pointIndex)
                {
                    ContactPoint& point =
                        manifold.Points.at(pointIndex);

                    const std::uint32_t stablePointIndex =
                        static_cast<std::uint32_t>(pointIndex);

                    const auto found =
                        std::find_if(
                            m_ContactCache.begin(),
                            m_ContactCache.end(),
                            [&manifold, stablePointIndex](const ContactCacheEntry& entry)
                            {
                                return SameCacheKey(entry, manifold, stablePointIndex);
                            });

                    if (found != m_ContactCache.end())
                    {
                        point.NormalImpulse = found->NormalImpulse;
                        point.TangentImpulse = found->TangentImpulse;
                    }
                }
            }
        }

        void StoreContactCache()
        {
            m_ContactCache.clear();

            for (const ContactManifold& manifold : m_LastContacts)
            {
                if (manifold.IsTrigger)
                {
                    continue;
                }

                for (std::size_t pointIndex = 0; pointIndex < manifold.Points.size(); ++pointIndex)
                {
                    const ContactPoint& point =
                        manifold.Points.at(pointIndex);

                    if (point.NormalImpulse <= 0.0f &&
                        std::abs(point.TangentImpulse) <= 1.0e-8f)
                    {
                        continue;
                    }

                    m_ContactCache.push_back(
                        ContactCacheEntry
                        {
                            manifold.BodyA,
                            manifold.BodyB,
                            manifold.ColliderA,
                            manifold.ColliderB,
                            static_cast<std::uint32_t>(pointIndex),
                            point.NormalImpulse,
                            point.TangentImpulse
                        });
                }
            }

            std::sort(
                m_ContactCache.begin(),
                m_ContactCache.end(),
                [](const ContactCacheEntry& lhs, const ContactCacheEntry& rhs)
                {
                    if (lhs.BodyA != rhs.BodyA)
                    {
                        return lhs.BodyA < rhs.BodyA;
                    }
                    if (lhs.BodyB != rhs.BodyB)
                    {
                        return lhs.BodyB < rhs.BodyB;
                    }
                    if (lhs.ColliderA != rhs.ColliderA)
                    {
                        return lhs.ColliderA < rhs.ColliderA;
                    }
                    if (lhs.ColliderB != rhs.ColliderB)
                    {
                        return lhs.ColliderB < rhs.ColliderB;
                    }
                    return lhs.PointIndex < rhs.PointIndex;
                });
        }
    };
}
