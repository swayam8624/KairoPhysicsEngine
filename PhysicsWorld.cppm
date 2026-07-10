module;

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.World;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;
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

    class PhysicsWorld final
    {
    public:
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
            ValidateStepSettings(Settings, dt);
            RequireFinite(Gravity, "Gravity");

            IntegrateForces(dt);
            IntegrateKinematicVelocities(dt);

            m_Broadphase.Sync(m_Bodies, m_Colliders);
            m_LastPairs =
                m_Broadphase.ComputePairs(m_Bodies, m_Colliders);

            m_LastContacts =
                ComputeContacts(m_Bodies, m_Colliders, m_LastPairs);

            UpdateContactEvents();
            WakeContactBodies();
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

            IntegrateDynamicVelocities(dt);
            UpdateSleeping(dt);
            ClearForceAccumulators();
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
        std::vector<DebugAABB> DebugAABBs() const
        {
            return CollectDebugAABBs(m_Bodies, m_Colliders);
        }

        [[nodiscard]]
        std::vector<DebugContact> DebugContacts() const
        {
            return CollectDebugContacts(m_LastContacts);
        }

        [[nodiscard]]
        std::vector<ColliderID> QueryAABB(
            const AABBf& query,
            std::uint32_t layerMask = 0xFFFF'FFFFu) const
        {
            std::vector<ColliderID> result;
            for (const Collider& collider : m_Colliders)
            {
                if (!IsValidCollider(collider.ID) ||
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

            std::vector<ColliderID> result;
            const float radiusSq =
                radius * radius;

            for (const Collider& collider : m_Colliders)
            {
                if (!IsValidCollider(collider.ID) ||
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

            std::vector<PhysicsRayHit> result;
            for (const Collider& collider : m_Colliders)
            {
                if (!IsValidCollider(collider.ID) ||
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

    private:
        std::vector<RigidBody> m_Bodies;
        std::vector<Collider> m_Colliders;
        BroadphaseWorld m_Broadphase;
        std::vector<BroadphasePair> m_LastPairs;
        std::vector<ContactManifold> m_LastContacts;
        std::vector<PhysicsContactEvent> m_LastEvents;
        float m_FixedAccumulator = 0.0f;

        struct ContactEventKey final
        {
            BodyID BodyA = InvalidBodyID;
            BodyID BodyB = InvalidBodyID;
            ColliderID ColliderA = InvalidColliderID;
            ColliderID ColliderB = InvalidColliderID;
            bool IsTrigger = false;
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
        static ContactEventKey MakeContactEventKey(
            const ContactManifold& manifold) noexcept
        {
            return
            {
                manifold.BodyA,
                manifold.BodyB,
                manifold.ColliderA,
                manifold.ColliderB,
                manifold.IsTrigger
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
                lhs.IsTrigger == rhs.IsTrigger;
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
            return lhs.IsTrigger < rhs.IsTrigger;
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
                        ContactEventKey{ lhs.BodyA, lhs.BodyB, lhs.ColliderA, lhs.ColliderB, lhs.IsTrigger },
                        ContactEventKey{ rhs.BodyA, rhs.BodyB, rhs.ColliderA, rhs.ColliderB, rhs.IsTrigger });
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
