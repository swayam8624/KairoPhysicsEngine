from pathlib import Path
import re

JOINT = r'''module;

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <variant>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.Joint;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Types;
import Kairo.Foundation.PhysicsEngine.RigidBody;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    using JointID = std::uint32_t;
    inline constexpr JointID InvalidJointID = std::numeric_limits<JointID>::max();

    struct DistanceJoint final
    {
        Vec3f LocalAnchorA = Vec3f::Zero();
        Vec3f LocalAnchorB = Vec3f::Zero();
        float RestLength = 0.0f;
    };

    struct BallSocketJoint final
    {
        Vec3f LocalAnchorA = Vec3f::Zero();
        Vec3f LocalAnchorB = Vec3f::Zero();
    };

    struct FixedJoint final
    {
        Vec3f LocalAnchorA = Vec3f::Zero();
        Vec3f LocalAnchorB = Vec3f::Zero();
        Quaternionf ReferenceRotation = Quaternionf::Identity();
    };

    struct HingeJoint final
    {
        Vec3f LocalAnchorA = Vec3f::Zero();
        Vec3f LocalAnchorB = Vec3f::Zero();
        Vec3f LocalAxisA = Vec3f::UnitY();
        Vec3f LocalAxisB = Vec3f::UnitY();
    };

    using JointConstraint =
        std::variant<DistanceJoint, BallSocketJoint, FixedJoint, HingeJoint>;

    struct Joint final
    {
        JointID ID = InvalidJointID;
        bool Active = true;
        BodyID BodyA = InvalidBodyID;
        BodyID BodyB = InvalidBodyID;
        bool CollideConnected = false;
        JointConstraint Constraint = DistanceJoint{};
    };

    [[nodiscard]]
    inline bool IsActiveJoint(const Joint& joint) noexcept
    {
        return joint.Active &&
            joint.ID != InvalidJointID &&
            joint.BodyA != InvalidBodyID &&
            joint.BodyB != InvalidBodyID &&
            joint.BodyA != joint.BodyB;
    }

    [[nodiscard]]
    inline Joint MakeDistanceJoint(
        JointID id,
        BodyID bodyA,
        BodyID bodyB,
        const Vec3f& localAnchorA,
        const Vec3f& localAnchorB,
        float restLength,
        bool collideConnected = false)
    {
        RequireFinite(localAnchorA, "DistanceJoint.LocalAnchorA");
        RequireFinite(localAnchorB, "DistanceJoint.LocalAnchorB");
        RequireNonNegative(restLength, "DistanceJoint.RestLength");
        return { id, true, bodyA, bodyB, collideConnected,
            DistanceJoint{ localAnchorA, localAnchorB, restLength } };
    }

    [[nodiscard]]
    inline Joint MakeBallSocketJoint(
        JointID id,
        BodyID bodyA,
        BodyID bodyB,
        const Vec3f& localAnchorA,
        const Vec3f& localAnchorB,
        bool collideConnected = false)
    {
        RequireFinite(localAnchorA, "BallSocketJoint.LocalAnchorA");
        RequireFinite(localAnchorB, "BallSocketJoint.LocalAnchorB");
        return { id, true, bodyA, bodyB, collideConnected,
            BallSocketJoint{ localAnchorA, localAnchorB } };
    }

    [[nodiscard]]
    inline Joint MakeFixedJoint(
        JointID id,
        BodyID bodyA,
        BodyID bodyB,
        const Vec3f& localAnchorA,
        const Vec3f& localAnchorB,
        const Quaternionf& referenceRotation,
        bool collideConnected = false)
    {
        RequireFinite(localAnchorA, "FixedJoint.LocalAnchorA");
        RequireFinite(localAnchorB, "FixedJoint.LocalAnchorB");
        return { id, true, bodyA, bodyB, collideConnected,
            FixedJoint{ localAnchorA, localAnchorB, referenceRotation.Normalized() } };
    }

    [[nodiscard]]
    inline Joint MakeHingeJoint(
        JointID id,
        BodyID bodyA,
        BodyID bodyB,
        const Vec3f& localAnchorA,
        const Vec3f& localAnchorB,
        const Vec3f& localAxisA,
        const Vec3f& localAxisB,
        bool collideConnected = false)
    {
        RequireFinite(localAnchorA, "HingeJoint.LocalAnchorA");
        RequireFinite(localAnchorB, "HingeJoint.LocalAnchorB");
        RequireFinite(localAxisA, "HingeJoint.LocalAxisA");
        RequireFinite(localAxisB, "HingeJoint.LocalAxisB");
        if (localAxisA.LengthSquared() <= 1.0e-10f ||
            localAxisB.LengthSquared() <= 1.0e-10f)
        {
            throw std::invalid_argument("HingeJoint axes must be non-zero.");
        }
        return { id, true, bodyA, bodyB, collideConnected,
            HingeJoint{
                localAnchorA,
                localAnchorB,
                localAxisA.Normalized(),
                localAxisB.Normalized()
            } };
    }

    [[nodiscard]]
    inline Vec3f WorldJointAnchor(
        const RigidBody& body,
        const Vec3f& localAnchor)
    {
        return body.State.Position + Rotate(body.State.Rotation, localAnchor);
    }

    [[nodiscard]]
    inline Vec3f FixedJointAngularError(
        const RigidBody& bodyA,
        const RigidBody& bodyB,
        const Quaternionf& referenceRotation)
    {
        const Quaternionf desiredB =
            (bodyA.State.Rotation * referenceRotation).Normalized();
        Quaternionf error =
            (Inverse(desiredB) * bodyB.State.Rotation).Normalized();
        if (error.w < 0.0f)
        {
            error = -error;
        }

        const Vec3f vectorPart = error.VectorPart();
        const float sineHalf = vectorPart.Length();
        if (sineHalf <= 1.0e-7f)
        {
            return Vec3f::Zero();
        }

        const float angle =
            2.0f * std::atan2(sineHalf, std::clamp(error.w, -1.0f, 1.0f));
        const Vec3f localAxis = vectorPart / sineHalf;
        return Rotate(desiredB, localAxis) * angle;
    }

    [[nodiscard]]
    inline Vec3f AxisAlignmentError(
        const Vec3f& axisA,
        const Vec3f& axisB)
    {
        const Vec3f a = SafeNormalize(axisA, Vec3f::UnitY());
        const Vec3f b = SafeNormalize(axisB, Vec3f::UnitY());
        const Vec3f cross = Cross(a, b);
        const float sine = cross.Length();
        if (sine <= 1.0e-7f)
        {
            if (Dot(a, b) >= 0.0f)
            {
                return Vec3f::Zero();
            }
            const Vec3f fallback =
                std::abs(a.y) < 0.9f ? Vec3f::Up() : Vec3f::UnitX();
            return SafeNormalize(Cross(a, fallback), Vec3f::UnitZ()) * Pi;
        }
        const float cosine = std::clamp(Dot(a, b), -1.0f, 1.0f);
        return (cross / sine) * std::atan2(sine, cosine);
    }

    namespace joint_detail
    {
        [[nodiscard]]
        inline float DynamicInverseMass(const RigidBody& body) noexcept
        {
            return IsDynamic(body) ? body.Mass.InverseMass : 0.0f;
        }

        inline void ApplyLinearAxisConstraint(
            RigidBody& bodyA,
            RigidBody& bodyB,
            const Vec3f& anchorA,
            const Vec3f& anchorB,
            const Vec3f& axis,
            float positionalError,
            const PhysicsStepSettings& settings,
            float dt)
        {
            const Vec3f n = SafeNormalize(axis, Vec3f::UnitX());
            const float invMassA = DynamicInverseMass(bodyA);
            const float invMassB = DynamicInverseMass(bodyB);
            const Matrix3f invInertiaA = WorldInverseInertia(bodyA);
            const Matrix3f invInertiaB = WorldInverseInertia(bodyB);
            const Vec3f rA = anchorA - bodyA.State.Position;
            const Vec3f rB = anchorB - bodyB.State.Position;
            const float denominator = ContactNormalDenominator(
                invMassA, invInertiaA, rA,
                invMassB, invInertiaB, rB,
                n);
            if (denominator <= 1.0e-7f)
            {
                return;
            }

            const float relativeVelocity = Dot(
                VelocityAtPoint(bodyB.State, anchorB) -
                VelocityAtPoint(bodyA.State, anchorA), n);
            const float maxBias = settings.MaxPositionCorrection / dt;
            const float bias = std::clamp(
                settings.Baumgarte * positionalError / dt,
                -maxBias,
                maxBias);
            const float lambda = -(relativeVelocity + bias) / denominator;
            const Vec3f impulse = n * lambda;

            if (invMassA > 0.0f)
            {
                ApplyImpulseAtPoint(
                    bodyA.State, invMassA, invInertiaA, -impulse, anchorA);
            }
            if (invMassB > 0.0f)
            {
                ApplyImpulseAtPoint(
                    bodyB.State, invMassB, invInertiaB, impulse, anchorB);
            }
        }

        inline void ApplyPointConstraint(
            RigidBody& bodyA,
            RigidBody& bodyB,
            const Vec3f& anchorA,
            const Vec3f& anchorB,
            const PhysicsStepSettings& settings,
            float dt)
        {
            const Vec3f error = anchorB - anchorA;
            ApplyLinearAxisConstraint(bodyA, bodyB, anchorA, anchorB,
                Vec3f::UnitX(), error.x, settings, dt);
            ApplyLinearAxisConstraint(bodyA, bodyB, anchorA, anchorB,
                Vec3f::UnitY(), error.y, settings, dt);
            ApplyLinearAxisConstraint(bodyA, bodyB, anchorA, anchorB,
                Vec3f::UnitZ(), error.z, settings, dt);
        }

        inline void ApplyAngularAxisConstraint(
            RigidBody& bodyA,
            RigidBody& bodyB,
            const Vec3f& axis,
            float angularError,
            const PhysicsStepSettings& settings,
            float dt)
        {
            const Vec3f n = SafeNormalize(axis, Vec3f::UnitX());
            const Matrix3f invInertiaA = WorldInverseInertia(bodyA);
            const Matrix3f invInertiaB = WorldInverseInertia(bodyB);
            const float denominator =
                Dot(n, invInertiaA * n + invInertiaB * n);
            if (denominator <= 1.0e-7f)
            {
                return;
            }

            const float relativeVelocity =
                Dot(bodyB.State.AngularVelocity - bodyA.State.AngularVelocity, n);
            constexpr float maxAngularBias = 20.0f;
            const float bias = std::clamp(
                settings.Baumgarte * angularError / dt,
                -maxAngularBias,
                maxAngularBias);
            const float lambda = -(relativeVelocity + bias) / denominator;
            const Vec3f impulse = n * lambda;

            if (IsDynamic(bodyA))
            {
                ApplyAngularImpulse(bodyA.State.AngularVelocity, invInertiaA, -impulse);
            }
            if (IsDynamic(bodyB))
            {
                ApplyAngularImpulse(bodyB.State.AngularVelocity, invInertiaB, impulse);
            }
        }

        inline void ApplyAngularVectorConstraint(
            RigidBody& bodyA,
            RigidBody& bodyB,
            const Vec3f& error,
            const PhysicsStepSettings& settings,
            float dt)
        {
            ApplyAngularAxisConstraint(bodyA, bodyB, Vec3f::UnitX(), error.x, settings, dt);
            ApplyAngularAxisConstraint(bodyA, bodyB, Vec3f::UnitY(), error.y, settings, dt);
            ApplyAngularAxisConstraint(bodyA, bodyB, Vec3f::UnitZ(), error.z, settings, dt);
        }

        inline void CorrectLinearError(
            RigidBody& bodyA,
            RigidBody& bodyB,
            const Vec3f& correction,
            const PhysicsStepSettings& settings)
        {
            const float invMassA = IsDynamic(bodyA) ? bodyA.Mass.InverseMass : 0.0f;
            const float invMassB = IsDynamic(bodyB) ? bodyB.Mass.InverseMass : 0.0f;
            const float sum = invMassA + invMassB;
            if (sum <= 1.0e-8f)
            {
                return;
            }

            Vec3f limited = correction;
            const float length = limited.Length();
            if (length > settings.MaxPositionCorrection && length > 1.0e-8f)
            {
                limited *= settings.MaxPositionCorrection / length;
            }
            limited /= static_cast<float>(settings.PositionIterations);

            if (invMassA > 0.0f)
            {
                bodyA.State.Position += limited * (invMassA / sum);
            }
            if (invMassB > 0.0f)
            {
                bodyB.State.Position -= limited * (invMassB / sum);
            }
        }

        inline void ApplyOrientationDelta(
            RigidBody& body,
            const Vec3f& rotationVector)
        {
            if (!IsDynamic(body))
            {
                return;
            }
            const float angle = rotationVector.Length();
            if (angle <= 1.0e-8f)
            {
                return;
            }
            const Quaternionf delta = AxisAngle(rotationVector / angle, angle);
            body.State.Rotation = (delta * body.State.Rotation).Normalized();
        }

        inline void CorrectAngularError(
            RigidBody& bodyA,
            RigidBody& bodyB,
            const Vec3f& error,
            const PhysicsStepSettings& settings)
        {
            float angle = error.Length();
            if (angle <= 1.0e-7f)
            {
                return;
            }
            const Vec3f axis = error / angle;
            constexpr float maxAngularCorrection = 0.25f;
            angle = std::min(angle, maxAngularCorrection) /
                static_cast<float>(settings.PositionIterations);

            const Matrix3f invInertiaA = WorldInverseInertia(bodyA);
            const Matrix3f invInertiaB = WorldInverseInertia(bodyB);
            const float mobilityA = IsDynamic(bodyA) ? Dot(axis, invInertiaA * axis) : 0.0f;
            const float mobilityB = IsDynamic(bodyB) ? Dot(axis, invInertiaB * axis) : 0.0f;
            const float sum = mobilityA + mobilityB;
            if (sum <= 1.0e-8f)
            {
                return;
            }

            ApplyOrientationDelta(bodyA, axis * (angle * mobilityA / sum));
            ApplyOrientationDelta(bodyB, axis * (-angle * mobilityB / sum));
        }
    }

    inline void SolveJointVelocity(
        std::vector<RigidBody>& bodies,
        Joint& joint,
        const PhysicsStepSettings& settings,
        float dt)
    {
        using namespace joint_detail;
        if (!IsActiveJoint(joint) ||
            joint.BodyA >= bodies.size() || joint.BodyB >= bodies.size())
        {
            return;
        }

        RigidBody& bodyA = bodies.at(joint.BodyA);
        RigidBody& bodyB = bodies.at(joint.BodyB);
        if (!IsActiveBody(bodyA) || !IsActiveBody(bodyB) ||
            (!IsDynamic(bodyA) && !IsDynamic(bodyB)))
        {
            return;
        }

        if (const auto* distance = std::get_if<DistanceJoint>(&joint.Constraint))
        {
            const Vec3f anchorA = WorldJointAnchor(bodyA, distance->LocalAnchorA);
            const Vec3f anchorB = WorldJointAnchor(bodyB, distance->LocalAnchorB);
            const Vec3f delta = anchorB - anchorA;
            const float length = delta.Length();
            const Vec3f axis = length > 1.0e-7f ? delta / length : Vec3f::UnitX();
            ApplyLinearAxisConstraint(
                bodyA, bodyB, anchorA, anchorB, axis,
                length - distance->RestLength, settings, dt);
            return;
        }

        if (const auto* ball = std::get_if<BallSocketJoint>(&joint.Constraint))
        {
            ApplyPointConstraint(
                bodyA, bodyB,
                WorldJointAnchor(bodyA, ball->LocalAnchorA),
                WorldJointAnchor(bodyB, ball->LocalAnchorB),
                settings, dt);
            return;
        }

        if (const auto* fixed = std::get_if<FixedJoint>(&joint.Constraint))
        {
            ApplyPointConstraint(
                bodyA, bodyB,
                WorldJointAnchor(bodyA, fixed->LocalAnchorA),
                WorldJointAnchor(bodyB, fixed->LocalAnchorB),
                settings, dt);
            ApplyAngularVectorConstraint(
                bodyA, bodyB,
                FixedJointAngularError(bodyA, bodyB, fixed->ReferenceRotation),
                settings, dt);
            return;
        }

        const auto& hinge = std::get<HingeJoint>(joint.Constraint);
        ApplyPointConstraint(
            bodyA, bodyB,
            WorldJointAnchor(bodyA, hinge.LocalAnchorA),
            WorldJointAnchor(bodyB, hinge.LocalAnchorB),
            settings, dt);

        const Vec3f axisA = Rotate(bodyA.State.Rotation, hinge.LocalAxisA).Normalized();
        const Vec3f axisB = Rotate(bodyB.State.Rotation, hinge.LocalAxisB).Normalized();
        const Vec3f hingeAxis = SafeNormalize(axisA + axisB, axisA);
        const Vec3f fallback =
            std::abs(hingeAxis.y) < 0.9f ? Vec3f::Up() : Vec3f::UnitX();
        const Vec3f tangentA = SafeNormalize(Cross(hingeAxis, fallback), Vec3f::UnitZ());
        const Vec3f tangentB = Cross(hingeAxis, tangentA).Normalized();
        const Vec3f error = AxisAlignmentError(axisA, axisB);
        ApplyAngularAxisConstraint(
            bodyA, bodyB, tangentA, Dot(error, tangentA), settings, dt);
        ApplyAngularAxisConstraint(
            bodyA, bodyB, tangentB, Dot(error, tangentB), settings, dt);
    }

    inline void CorrectJointPosition(
        std::vector<RigidBody>& bodies,
        const Joint& joint,
        const PhysicsStepSettings& settings)
    {
        using namespace joint_detail;
        if (!IsActiveJoint(joint) ||
            joint.BodyA >= bodies.size() || joint.BodyB >= bodies.size())
        {
            return;
        }

        RigidBody& bodyA = bodies.at(joint.BodyA);
        RigidBody& bodyB = bodies.at(joint.BodyB);
        if (!IsActiveBody(bodyA) || !IsActiveBody(bodyB))
        {
            return;
        }

        if (const auto* distance = std::get_if<DistanceJoint>(&joint.Constraint))
        {
            const Vec3f anchorA = WorldJointAnchor(bodyA, distance->LocalAnchorA);
            const Vec3f anchorB = WorldJointAnchor(bodyB, distance->LocalAnchorB);
            const Vec3f delta = anchorB - anchorA;
            const float length = delta.Length();
            if (length > 1.0e-7f)
            {
                CorrectLinearError(
                    bodyA, bodyB,
                    (delta / length) * (length - distance->RestLength),
                    settings);
            }
            return;
        }

        if (const auto* ball = std::get_if<BallSocketJoint>(&joint.Constraint))
        {
            CorrectLinearError(
                bodyA, bodyB,
                WorldJointAnchor(bodyB, ball->LocalAnchorB) -
                    WorldJointAnchor(bodyA, ball->LocalAnchorA),
                settings);
            return;
        }

        if (const auto* fixed = std::get_if<FixedJoint>(&joint.Constraint))
        {
            CorrectLinearError(
                bodyA, bodyB,
                WorldJointAnchor(bodyB, fixed->LocalAnchorB) -
                    WorldJointAnchor(bodyA, fixed->LocalAnchorA),
                settings);
            CorrectAngularError(
                bodyA, bodyB,
                FixedJointAngularError(bodyA, bodyB, fixed->ReferenceRotation),
                settings);
            return;
        }

        const auto& hinge = std::get<HingeJoint>(joint.Constraint);
        CorrectLinearError(
            bodyA, bodyB,
            WorldJointAnchor(bodyB, hinge.LocalAnchorB) -
                WorldJointAnchor(bodyA, hinge.LocalAnchorA),
            settings);
        const Vec3f axisA = Rotate(bodyA.State.Rotation, hinge.LocalAxisA);
        const Vec3f axisB = Rotate(bodyB.State.Rotation, hinge.LocalAxisB);
        CorrectAngularError(bodyA, bodyB, AxisAlignmentError(axisA, axisB), settings);
    }
}
'''

ISLAND = r'''module;

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.SolverIsland;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine.Types;
import Kairo.Foundation.PhysicsEngine.RigidBody;
import Kairo.Foundation.PhysicsEngine.Collider;
import Kairo.Foundation.PhysicsEngine.ContactSolver;
import Kairo.Foundation.PhysicsEngine.Joint;

export namespace kairo::foundation::physics
{
    struct SolverIsland final
    {
        std::vector<BodyID> DynamicBodies;
        std::vector<std::size_t> ContactIndices;
        std::vector<std::size_t> JointIndices;
    };

    namespace island_detail
    {
        class DisjointSet final
        {
        public:
            explicit DisjointSet(std::size_t count)
                : m_Parent(count)
                , m_Rank(count, 0u)
            {
                for (std::size_t i = 0; i < count; ++i)
                {
                    m_Parent[i] = i;
                }
            }

            [[nodiscard]]
            std::size_t Find(std::size_t value)
            {
                if (m_Parent[value] != value)
                {
                    m_Parent[value] = Find(m_Parent[value]);
                }
                return m_Parent[value];
            }

            void Unite(std::size_t a, std::size_t b)
            {
                a = Find(a);
                b = Find(b);
                if (a == b)
                {
                    return;
                }
                if (m_Rank[a] < m_Rank[b])
                {
                    std::swap(a, b);
                }
                m_Parent[b] = a;
                if (m_Rank[a] == m_Rank[b])
                {
                    ++m_Rank[a];
                }
            }

        private:
            std::vector<std::size_t> m_Parent;
            std::vector<std::uint8_t> m_Rank;
        };

        inline void CorrectContactOnce(
            std::vector<RigidBody>& bodies,
            const ContactManifold& manifold,
            const PhysicsStepSettings& settings)
        {
            if (manifold.IsTrigger ||
                manifold.BodyA >= bodies.size() || manifold.BodyB >= bodies.size())
            {
                return;
            }
            RigidBody& bodyA = bodies.at(manifold.BodyA);
            RigidBody& bodyB = bodies.at(manifold.BodyB);
            if (!IsActiveBody(bodyA) || !IsActiveBody(bodyB))
            {
                return;
            }

            const float invMassA = IsDynamic(bodyA) ? bodyA.Mass.InverseMass : 0.0f;
            const float invMassB = IsDynamic(bodyB) ? bodyB.Mass.InverseMass : 0.0f;
            const float inverseMassSum = invMassA + invMassB;
            if (inverseMassSum <= 0.0f)
            {
                return;
            }

            for (const ContactPoint& point : manifold.Points)
            {
                const float totalCorrectionMagnitude = std::min(
                    std::max(point.PenetrationDepth - settings.Slop, 0.0f) /
                        inverseMassSum,
                    settings.MaxPositionCorrection);
                const float correctionMagnitude =
                    totalCorrectionMagnitude /
                    static_cast<float>(settings.PositionIterations);
                const Vec3f correction =
                    SafeNormalize(point.Normal, Vec3f::Up()) * correctionMagnitude;
                if (invMassA > 0.0f)
                {
                    bodyA.State.Position -= correction * invMassA;
                }
                if (invMassB > 0.0f)
                {
                    bodyB.State.Position += correction * invMassB;
                }
            }
        }

        inline void SolveIslandVelocity(
            std::vector<RigidBody>& bodies,
            const std::vector<Collider>& colliders,
            std::vector<ContactManifold>& contacts,
            std::vector<Joint>& joints,
            const SolverIsland& island,
            const PhysicsStepSettings& settings,
            float dt)
        {
            for (std::uint32_t iteration = 0;
                iteration < settings.VelocityIterations; ++iteration)
            {
                for (const std::size_t contactIndex : island.ContactIndices)
                {
                    SolveContactManifold(
                        bodies, colliders, contacts.at(contactIndex), settings, dt);
                }
                for (const std::size_t jointIndex : island.JointIndices)
                {
                    SolveJointVelocity(
                        bodies, joints.at(jointIndex), settings, dt);
                }
            }
        }

        inline void CorrectIslandPositions(
            std::vector<RigidBody>& bodies,
            const std::vector<ContactManifold>& contacts,
            const std::vector<Joint>& joints,
            const SolverIsland& island,
            const PhysicsStepSettings& settings)
        {
            for (std::uint32_t iteration = 0;
                iteration < settings.PositionIterations; ++iteration)
            {
                for (const std::size_t contactIndex : island.ContactIndices)
                {
                    CorrectContactOnce(bodies, contacts.at(contactIndex), settings);
                }
                for (const std::size_t jointIndex : island.JointIndices)
                {
                    CorrectJointPosition(bodies, joints.at(jointIndex), settings);
                }
            }
        }

        template<typename Function>
        inline void DispatchIslands(
            const std::vector<SolverIsland>& islands,
            bool parallel,
            std::uint32_t minimumParallelCount,
            Function function)
        {
            if (!parallel ||
                islands.size() < static_cast<std::size_t>(minimumParallelCount) ||
                islands.size() < 2u)
            {
                for (std::size_t i = 0; i < islands.size(); ++i)
                {
                    function(i);
                }
                return;
            }

            const unsigned reported = std::thread::hardware_concurrency();
            const std::size_t workerCount = std::min<std::size_t>(
                islands.size(), reported == 0u ? 2u : reported);
            if (workerCount <= 1u)
            {
                for (std::size_t i = 0; i < islands.size(); ++i)
                {
                    function(i);
                }
                return;
            }

            std::atomic<std::size_t> next{ 0u };
            std::vector<std::jthread> workers;
            workers.reserve(workerCount);
            for (std::size_t worker = 0; worker < workerCount; ++worker)
            {
                workers.emplace_back(
                    [&]()
                    {
                        while (true)
                        {
                            const std::size_t index = next.fetch_add(1u);
                            if (index >= islands.size())
                            {
                                return;
                            }
                            function(index);
                        }
                    });
            }
        }
    }

    [[nodiscard]]
    inline std::vector<SolverIsland> BuildSolverIslands(
        const std::vector<RigidBody>& bodies,
        const std::vector<ContactManifold>& contacts,
        const std::vector<Joint>& joints)
    {
        using namespace island_detail;
        DisjointSet sets(bodies.size());

        for (const ContactManifold& contact : contacts)
        {
            if (contact.IsTrigger ||
                contact.BodyA >= bodies.size() || contact.BodyB >= bodies.size())
            {
                continue;
            }
            if (IsDynamicBodyType(bodies.at(contact.BodyA)) &&
                IsDynamicBodyType(bodies.at(contact.BodyB)))
            {
                sets.Unite(contact.BodyA, contact.BodyB);
            }
        }

        for (const Joint& joint : joints)
        {
            if (!IsActiveJoint(joint) ||
                joint.BodyA >= bodies.size() || joint.BodyB >= bodies.size())
            {
                continue;
            }
            if (IsDynamicBodyType(bodies.at(joint.BodyA)) &&
                IsDynamicBodyType(bodies.at(joint.BodyB)))
            {
                sets.Unite(joint.BodyA, joint.BodyB);
            }
        }

        const std::size_t none = bodies.size();
        std::vector<std::size_t> rootToIsland(bodies.size(), none);
        std::vector<SolverIsland> islands;

        for (const RigidBody& body : bodies)
        {
            if (!IsDynamicBodyType(body))
            {
                continue;
            }
            const std::size_t root = sets.Find(body.ID);
            if (rootToIsland[root] == none)
            {
                rootToIsland[root] = islands.size();
                islands.push_back({});
            }
            islands[rootToIsland[root]].DynamicBodies.push_back(body.ID);
        }

        const auto islandForBodies =
            [&](BodyID bodyA, BodyID bodyB) -> std::size_t
            {
                if (bodyA < bodies.size() && IsDynamicBodyType(bodies.at(bodyA)))
                {
                    return rootToIsland[sets.Find(bodyA)];
                }
                if (bodyB < bodies.size() && IsDynamicBodyType(bodies.at(bodyB)))
                {
                    return rootToIsland[sets.Find(bodyB)];
                }
                return none;
            };

        for (std::size_t index = 0; index < contacts.size(); ++index)
        {
            const ContactManifold& contact = contacts[index];
            if (contact.IsTrigger)
            {
                continue;
            }
            const std::size_t island = islandForBodies(contact.BodyA, contact.BodyB);
            if (island != none)
            {
                islands[island].ContactIndices.push_back(index);
            }
        }

        for (std::size_t index = 0; index < joints.size(); ++index)
        {
            const Joint& joint = joints[index];
            if (!IsActiveJoint(joint))
            {
                continue;
            }
            const std::size_t island = islandForBodies(joint.BodyA, joint.BodyB);
            if (island != none)
            {
                islands[island].JointIndices.push_back(index);
            }
        }

        return islands;
    }

    inline void SolveSolverIslands(
        std::vector<RigidBody>& bodies,
        const std::vector<Collider>& colliders,
        std::vector<ContactManifold>& contacts,
        std::vector<Joint>& joints,
        const std::vector<SolverIsland>& islands,
        const PhysicsStepSettings& settings,
        float dt)
    {
        island_detail::DispatchIslands(
            islands,
            settings.EnableParallelIslands,
            settings.ParallelIslandMinCount,
            [&](std::size_t index)
            {
                island_detail::SolveIslandVelocity(
                    bodies, colliders, contacts, joints,
                    islands[index], settings, dt);
            });
    }

    inline void CorrectSolverIslandPositions(
        std::vector<RigidBody>& bodies,
        const std::vector<ContactManifold>& contacts,
        const std::vector<Joint>& joints,
        const std::vector<SolverIsland>& islands,
        const PhysicsStepSettings& settings)
    {
        island_detail::DispatchIslands(
            islands,
            settings.EnableParallelIslands,
            settings.ParallelIslandMinCount,
            [&](std::size_t index)
            {
                island_detail::CorrectIslandPositions(
                    bodies, contacts, joints,
                    islands[index], settings);
            });
    }
}
'''

Path('Joint.cppm').write_text(JOINT)
Path('SolverIsland.cppm').write_text(ISLAND)

# PhysicsStepSettings: parallel island controls.
p = Path('PhysicsEngineTypes.cppm')
s = p.read_text()
old = '''        float MaxPositionCorrection = 0.25f;\n        bool EnableSleeping = true;\n'''
new = '''        float MaxPositionCorrection = 0.25f;\n        bool EnableParallelIslands = false;\n        std::uint32_t ParallelIslandMinCount = 4;\n        bool EnableSleeping = true;\n'''
if old not in s:
    raise SystemExit('PhysicsStepSettings insertion marker not found')
s = s.replace(old, new, 1)
old = '''        if (settings.PositionIterations == 0)\n        {\n            throw std::invalid_argument("PositionIterations must be greater than zero.");\n        }\n'''
new = old + '''\n        if (settings.ParallelIslandMinCount == 0)\n        {\n            throw std::invalid_argument("ParallelIslandMinCount must be greater than zero.");\n        }\n'''
if old not in s:
    raise SystemExit('PhysicsStepSettings validation marker not found')
s = s.replace(old, new, 1)
p.write_text(s)

# CMake: register modules and platform threads.
p = Path('CMakeLists.txt')
s = p.read_text()
if 'find_package(Threads REQUIRED)' not in s:
    s = s.replace('add_library(KairoPhysicsEngine)\n',
                  'find_package(Threads REQUIRED)\n\nadd_library(KairoPhysicsEngine)\n', 1)
old = '''            ContactSolver.cppm\n            PhysicsDebug.cppm\n            PhysicsWorld.cppm\n'''
new = '''            ContactSolver.cppm\n            Joint.cppm\n            SolverIsland.cppm\n            PhysicsDebug.cppm\n            PhysicsWorld.cppm\n'''
if old not in s:
    raise SystemExit('CMake module list marker not found')
s = s.replace(old, new, 1)
s = s.replace(
    'target_link_libraries(KairoPhysicsEngine PUBLIC KairoPhysicsMath KairoSpatial)',
    'target_link_libraries(KairoPhysicsEngine PUBLIC KairoPhysicsMath KairoSpatial Threads::Threads)',
    1)
p.write_text(s)

# Umbrella exports.
p = Path('KairoPhysicsEngine.cppm')
s = p.read_text()
old = '''export import Kairo.Foundation.PhysicsEngine.ContactSolver;\nexport import Kairo.Foundation.PhysicsEngine.Debug;\n'''
new = '''export import Kairo.Foundation.PhysicsEngine.ContactSolver;\nexport import Kairo.Foundation.PhysicsEngine.Joint;\nexport import Kairo.Foundation.PhysicsEngine.SolverIsland;\nexport import Kairo.Foundation.PhysicsEngine.Debug;\n'''
if old not in s:
    raise SystemExit('umbrella export marker not found')
s = s.replace(old, new, 1)
p.write_text(s)

# PhysicsWorld orchestration and APIs.
p = Path('PhysicsWorld.cppm')
s = p.read_text()
old = '''import Kairo.Foundation.PhysicsEngine.ContactSolver;\nimport Kairo.Foundation.PhysicsEngine.Debug;\n'''
new = '''import Kairo.Foundation.PhysicsEngine.ContactSolver;\nimport Kairo.Foundation.PhysicsEngine.Joint;\nimport Kairo.Foundation.PhysicsEngine.SolverIsland;\nimport Kairo.Foundation.PhysicsEngine.Debug;\n'''
if old not in s:
    raise SystemExit('world import marker not found')
s = s.replace(old, new, 1)

old = '''        [[nodiscard]]\n        bool IsValidCollider(\n            ColliderID collider) const noexcept\n        {\n            return collider < m_Colliders.size() &&\n                IsActiveCollider(m_Colliders[collider]) &&\n                IsValidBody(m_Colliders[collider].Body);\n        }\n\n'''
if old not in s:
    raise SystemExit('IsValidCollider block not found')
new = old + '''        [[nodiscard]]\n        bool IsValidJoint(\n            JointID joint) const noexcept\n        {\n            return joint < m_Joints.size() &&\n                IsActiveJoint(m_Joints[joint]) &&\n                IsValidBody(m_Joints[joint].BodyA) &&\n                IsValidBody(m_Joints[joint].BodyB);\n        }\n\n'''
s = s.replace(old, new, 1)

insert_marker = '''        /// Input: active body id.\n        /// Output: none.\n        /// Task: deactivate a body and every collider attached to it without\n'''
if insert_marker not in s:
    raise SystemExit('joint public API insertion marker not found')
joint_api = r'''        [[nodiscard]]
        JointID CreateDistanceJoint(
            BodyID bodyA,
            BodyID bodyB,
            const Vec3f& worldAnchorA,
            const Vec3f& worldAnchorB,
            float restLength = -1.0f,
            bool collideConnected = false)
        {
            RequireJointBodies(bodyA, bodyB, "CreateDistanceJoint");
            RequireFinite(worldAnchorA, "CreateDistanceJoint.worldAnchorA");
            RequireFinite(worldAnchorB, "CreateDistanceJoint.worldAnchorB");
            const float resolvedLength = restLength < 0.0f
                ? (worldAnchorB - worldAnchorA).Length()
                : restLength;
            RequireNonNegative(resolvedLength, "CreateDistanceJoint.restLength");
            const JointID id = static_cast<JointID>(m_Joints.size());
            m_Joints.push_back(MakeDistanceJoint(
                id, bodyA, bodyB,
                BodyLocalPoint(m_Bodies.at(bodyA), worldAnchorA),
                BodyLocalPoint(m_Bodies.at(bodyB), worldAnchorB),
                resolvedLength,
                collideConnected));
            WakeJointPair(bodyA, bodyB);
            return id;
        }

        [[nodiscard]]
        JointID CreateBallSocketJoint(
            BodyID bodyA,
            BodyID bodyB,
            const Vec3f& worldAnchor,
            bool collideConnected = false)
        {
            RequireJointBodies(bodyA, bodyB, "CreateBallSocketJoint");
            RequireFinite(worldAnchor, "CreateBallSocketJoint.worldAnchor");
            const JointID id = static_cast<JointID>(m_Joints.size());
            m_Joints.push_back(MakeBallSocketJoint(
                id, bodyA, bodyB,
                BodyLocalPoint(m_Bodies.at(bodyA), worldAnchor),
                BodyLocalPoint(m_Bodies.at(bodyB), worldAnchor),
                collideConnected));
            WakeJointPair(bodyA, bodyB);
            return id;
        }

        [[nodiscard]]
        JointID CreateFixedJoint(
            BodyID bodyA,
            BodyID bodyB,
            const Vec3f& worldAnchor,
            bool collideConnected = false)
        {
            RequireJointBodies(bodyA, bodyB, "CreateFixedJoint");
            RequireFinite(worldAnchor, "CreateFixedJoint.worldAnchor");
            const JointID id = static_cast<JointID>(m_Joints.size());
            const Quaternionf reference =
                (Inverse(m_Bodies.at(bodyA).State.Rotation) *
                    m_Bodies.at(bodyB).State.Rotation).Normalized();
            m_Joints.push_back(MakeFixedJoint(
                id, bodyA, bodyB,
                BodyLocalPoint(m_Bodies.at(bodyA), worldAnchor),
                BodyLocalPoint(m_Bodies.at(bodyB), worldAnchor),
                reference,
                collideConnected));
            WakeJointPair(bodyA, bodyB);
            return id;
        }

        [[nodiscard]]
        JointID CreateHingeJoint(
            BodyID bodyA,
            BodyID bodyB,
            const Vec3f& worldAnchor,
            const Vec3f& worldAxis,
            bool collideConnected = false)
        {
            RequireJointBodies(bodyA, bodyB, "CreateHingeJoint");
            RequireFinite(worldAnchor, "CreateHingeJoint.worldAnchor");
            RequireFinite(worldAxis, "CreateHingeJoint.worldAxis");
            if (worldAxis.LengthSquared() <= 1.0e-10f)
            {
                throw std::invalid_argument("CreateHingeJoint.worldAxis must be non-zero.");
            }
            const Vec3f axis = worldAxis.Normalized();
            const JointID id = static_cast<JointID>(m_Joints.size());
            m_Joints.push_back(MakeHingeJoint(
                id, bodyA, bodyB,
                BodyLocalPoint(m_Bodies.at(bodyA), worldAnchor),
                BodyLocalPoint(m_Bodies.at(bodyB), worldAnchor),
                Rotate(m_Bodies.at(bodyA).State.Rotation.Conjugate(), axis),
                Rotate(m_Bodies.at(bodyB).State.Rotation.Conjugate(), axis),
                collideConnected));
            WakeJointPair(bodyA, bodyB);
            return id;
        }

        void DestroyJoint(JointID joint)
        {
            if (joint >= m_Joints.size())
            {
                throw std::out_of_range("DestroyJoint failed: joint id does not exist.");
            }
            Joint& record = m_Joints.at(joint);
            if (!record.Active)
            {
                return;
            }
            record.Active = false;
            record.BodyA = InvalidBodyID;
            record.BodyB = InvalidBodyID;
        }

'''
s = s.replace(insert_marker, joint_api + insert_marker, 1)

# Body destruction removes attached joints first.
old = '''            for (Collider& collider : m_Colliders)\n            {\n                if (collider.Active && collider.Body == body)\n                {\n                    RemoveCollider(collider.ID);\n                }\n            }\n\n            record.Active = false;\n'''
new = '''            for (Collider& collider : m_Colliders)\n            {\n                if (collider.Active && collider.Body == body)\n                {\n                    RemoveCollider(collider.ID);\n                }\n            }\n\n            for (Joint& joint : m_Joints)\n            {\n                if (joint.Active && (joint.BodyA == body || joint.BodyB == body))\n                {\n                    DestroyJoint(joint.ID);\n                }\n            }\n\n            record.Active = false;\n'''
if old not in s:
    raise SystemExit('DestroyRigidBody collider loop marker not found')
s = s.replace(old, new, 1)

# Step: wake joints, build islands, solve contacts+joints through island dispatch.
old = '''            ApplyCollisionResponses();\n            UpdateContactEvents();\n            DispatchContactEvents();\n            WakeContactBodies();\n\n            const auto narrowphaseEnd =\n'''
new = '''            ApplyCollisionResponses();\n            UpdateContactEvents();\n            DispatchContactEvents();\n            WakeContactBodies();\n            WakeJointBodies();\n            m_LastIslands = BuildSolverIslands(m_Bodies, m_LastContacts, m_Joints);\n\n            const auto narrowphaseEnd =\n'''
if old not in s:
    raise SystemExit('Step wake marker not found')
s = s.replace(old, new, 1)

old = '''            RestoreContactCache();\n            WarmStartContacts(m_Bodies, m_Colliders, m_LastContacts);\n\n            SolveContacts(\n                m_Bodies,\n                m_Colliders,\n                m_LastContacts,\n                Settings,\n                dt);\n\n            StoreContactCache();\n\n            CorrectPositions(\n                m_Bodies,\n                m_LastContacts,\n                Settings);\n'''
new = '''            RestoreContactCache();\n            WarmStartContacts(m_Bodies, m_Colliders, m_LastContacts);\n\n            SolveSolverIslands(\n                m_Bodies,\n                m_Colliders,\n                m_LastContacts,\n                m_Joints,\n                m_LastIslands,\n                Settings,\n                dt);\n\n            StoreContactCache();\n\n            CorrectSolverIslandPositions(\n                m_Bodies,\n                m_LastContacts,\n                m_Joints,\n                m_LastIslands,\n                Settings);\n'''
if old not in s:
    raise SystemExit('Step solver block not found')
s = s.replace(old, new, 1)

# Public accessors.
old = '''        [[nodiscard]]\n        const std::vector<ContactManifold>& Contacts() const noexcept\n        {\n            return m_LastContacts;\n        }\n\n'''
if old not in s:
    raise SystemExit('Contacts accessor marker not found')
new = old + '''        [[nodiscard]]\n        const std::vector<Joint>& Joints() const noexcept\n        {\n            return m_Joints;\n        }\n\n        [[nodiscard]]\n        const std::vector<SolverIsland>& SolverIslands() const noexcept\n        {\n            return m_LastIslands;\n        }\n\n'''
s = s.replace(old, new, 1)

# Private storage.
old = '''        std::vector<RigidBody> m_Bodies;\n        std::vector<Collider> m_Colliders;\n        mutable BroadphaseWorld m_Broadphase;\n'''
new = '''        std::vector<RigidBody> m_Bodies;\n        std::vector<Collider> m_Colliders;\n        std::vector<Joint> m_Joints;\n        std::vector<SolverIsland> m_LastIslands;\n        mutable BroadphaseWorld m_Broadphase;\n'''
if old not in s:
    raise SystemExit('private storage marker not found')
s = s.replace(old, new, 1)

# Private joint helpers before SyncBroadphase.
marker = '''        /// Input: none; this is a logically-const maintenance operation.\n'''
if marker not in s:
    raise SystemExit('private helper insertion marker not found')
helpers = r'''        void RequireJointBodies(
            BodyID bodyA,
            BodyID bodyB,
            const char* operation) const
        {
            if (!IsValidBody(bodyA) || !IsValidBody(bodyB))
            {
                throw std::out_of_range(
                    std::string(operation) +
                    " failed: body id does not exist or is inactive.");
            }
            if (bodyA == bodyB)
            {
                throw std::invalid_argument(
                    std::string(operation) +
                    " failed: a joint must connect two different bodies.");
            }
        }

        [[nodiscard]]
        static Vec3f BodyLocalPoint(
            const RigidBody& body,
            const Vec3f& worldPoint)
        {
            return Rotate(
                body.State.Rotation.Conjugate(),
                worldPoint - body.State.Position);
        }

        void WakeJointPair(BodyID bodyA, BodyID bodyB)
        {
            if (bodyA < m_Bodies.size() && IsDynamicBodyType(m_Bodies.at(bodyA)))
            {
                WakeRigidBody(m_Bodies.at(bodyA));
            }
            if (bodyB < m_Bodies.size() && IsDynamicBodyType(m_Bodies.at(bodyB)))
            {
                WakeRigidBody(m_Bodies.at(bodyB));
            }
        }

        void WakeJointBodies()
        {
            for (const Joint& joint : m_Joints)
            {
                if (!IsActiveJoint(joint) ||
                    joint.BodyA >= m_Bodies.size() || joint.BodyB >= m_Bodies.size())
                {
                    continue;
                }
                RigidBody& bodyA = m_Bodies.at(joint.BodyA);
                RigidBody& bodyB = m_Bodies.at(joint.BodyB);

                const bool wakeA =
                    IsDynamicBodyType(bodyA) && bodyA.Sleeping &&
                    ((IsDynamicBodyType(bodyB) && !bodyB.Sleeping) ||
                     (IsKinematic(bodyB) &&
                      (bodyB.State.LinearVelocity.LengthSquared() > 1.0e-12f ||
                       bodyB.State.AngularVelocity.LengthSquared() > 1.0e-12f)));
                const bool wakeB =
                    IsDynamicBodyType(bodyB) && bodyB.Sleeping &&
                    ((IsDynamicBodyType(bodyA) && !bodyA.Sleeping) ||
                     (IsKinematic(bodyA) &&
                      (bodyA.State.LinearVelocity.LengthSquared() > 1.0e-12f ||
                       bodyA.State.AngularVelocity.LengthSquared() > 1.0e-12f)));
                if (wakeA) WakeRigidBody(bodyA);
                if (wakeB) WakeRigidBody(bodyB);
            }
        }

        [[nodiscard]]
        bool JointDisablesCollision(BodyID bodyA, BodyID bodyB) const noexcept
        {
            for (const Joint& joint : m_Joints)
            {
                if (!IsActiveJoint(joint) || joint.CollideConnected)
                {
                    continue;
                }
                if ((joint.BodyA == bodyA && joint.BodyB == bodyB) ||
                    (joint.BodyA == bodyB && joint.BodyB == bodyA))
                {
                    return true;
                }
            }
            return false;
        }

'''
s = s.replace(marker, helpers + marker, 1)

# Connected bodies default to no collision response.
old = '''            for (ContactManifold manifold : m_LastContacts)\n            {\n                if (manifold.ColliderA >= m_Colliders.size() ||\n                    manifold.ColliderB >= m_Colliders.size())\n                {\n                    continue;\n                }\n\n'''
new = '''            for (ContactManifold manifold : m_LastContacts)\n            {\n                if (manifold.ColliderA >= m_Colliders.size() ||\n                    manifold.ColliderB >= m_Colliders.size() ||\n                    JointDisablesCollision(manifold.BodyA, manifold.BodyB))\n                {\n                    continue;\n                }\n\n'''
if old not in s:
    raise SystemExit('ApplyCollisionResponses manifold marker not found')
s = s.replace(old, new, 1)
p.write_text(s)

# Terminal sandbox uses the engine's real island graph instead of a second DSU.
p = Path('sandbox/TerminalRenderer.cppm')
s = p.read_text()
pattern = re.compile(r'''        std::vector<std::size_t> parent\(world\.Bodies\(\)\.size\(\)\);\n        std::iota\(parent\.begin\(\), parent\.end\(\), std::size_t\{ 0 \}\);\n\n        for \(const ContactManifold& contact : world\.Contacts\(\)\)\n        \{.*?\n        stats\.Islands = roots\.size\(\);''', re.S)
if not pattern.search(s):
    raise SystemExit('sandbox island stats block not found')
s = pattern.sub(
    '        stats.Islands =\n            BuildSolverIslands(world.Bodies(), world.Contacts(), world.Joints()).size();',
    s, count=1)
p.write_text(s)

# Tests for every joint family, island topology, and parallel equivalence.
p = Path('tests/PhysicsEngineTests.cpp')
s = p.read_text()
if 'Distance joints preserve authored separation' not in s:
    tests = r'''
TEST_CASE("Distance joints preserve authored separation",
    "[PhysicsEngine][Joint][Distance]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;
    const BodyID anchor = world.CreateRigidBody(StaticBody());
    RigidBodyDesc dynamic = DynamicSphereBody(Vec3f{ 2.0f, 0.0f, 0.0f }, 0.25f);
    dynamic.State.LinearVelocity = Vec3f{ -8.0f, 3.0f, 0.0f };
    const BodyID bob = world.CreateRigidBody(dynamic);
    const JointID joint = world.CreateDistanceJoint(
        anchor, bob, Vec3f::Zero(), Vec3f{ 2.0f, 0.0f, 0.0f }, 2.0f);
    REQUIRE(world.IsValidJoint(joint));

    for (int i = 0; i < 180; ++i)
    {
        world.Step(1.0f / 120.0f);
    }

    CHECK(world.Bodies().at(bob).State.Position.Length() ==
        Catch::Approx(2.0f).margin(0.04f));
}

TEST_CASE("Ball socket joints keep a shared anchor coincident",
    "[PhysicsEngine][Joint][BallSocket]")
{
    PhysicsWorld world;
    world.Settings.EnableSleeping = false;
    const BodyID anchor = world.CreateRigidBody(StaticBody());
    const BodyID bob = world.CreateRigidBody(
        DynamicSphereBody(Vec3f{ 0.0f, -1.0f, 0.0f }, 0.25f));
    const JointID id = world.CreateBallSocketJoint(anchor, bob, Vec3f::Zero());

    for (int i = 0; i < 180; ++i)
    {
        world.Step(1.0f / 120.0f);
    }

    const auto& joint = std::get<BallSocketJoint>(world.Joints().at(id).Constraint);
    const Vec3f a = WorldJointAnchor(world.Bodies().at(anchor), joint.LocalAnchorA);
    const Vec3f b = WorldJointAnchor(world.Bodies().at(bob), joint.LocalAnchorB);
    CHECK((b - a).Length() < 0.04f);
}

TEST_CASE("Fixed joints preserve relative pose under authored velocity",
    "[PhysicsEngine][Joint][Fixed]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;
    const BodyID anchor = world.CreateRigidBody(StaticBody());
    RigidBodyDesc moving = DynamicBoxBody(Vec3f{ 1.0f, 0.0f, 0.0f });
    moving.State.LinearVelocity = Vec3f{ 5.0f, 2.0f, 0.0f };
    moving.State.AngularVelocity = Vec3f{ 2.0f, 3.0f, 4.0f };
    const BodyID body = world.CreateRigidBody(moving);
    [[maybe_unused]] const JointID id =
        world.CreateFixedJoint(anchor, body, Vec3f{ 1.0f, 0.0f, 0.0f });

    for (int i = 0; i < 120; ++i)
    {
        world.Step(1.0f / 120.0f);
    }

    CHECK((world.Bodies().at(body).State.Position - Vec3f{ 1.0f, 0.0f, 0.0f }).Length() < 0.05f);
    const Quaternionf rotation = world.Bodies().at(body).State.Rotation;
    CHECK(std::abs(rotation.x) < 0.04f);
    CHECK(std::abs(rotation.y) < 0.04f);
    CHECK(std::abs(rotation.z) < 0.04f);
}

TEST_CASE("Hinge joints preserve the free axis while constraining orthogonal spin",
    "[PhysicsEngine][Joint][Hinge]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;
    const BodyID anchor = world.CreateRigidBody(StaticBody());
    RigidBodyDesc moving = DynamicBoxBody(Vec3f::Zero());
    moving.State.AngularVelocity = Vec3f{ 5.0f, 5.0f, 0.0f };
    const BodyID body = world.CreateRigidBody(moving);
    [[maybe_unused]] const JointID id =
        world.CreateHingeJoint(anchor, body, Vec3f::Zero(), Vec3f::UnitY());

    for (int i = 0; i < 30; ++i)
    {
        world.Step(1.0f / 120.0f);
    }

    CHECK(std::abs(world.Bodies().at(body).State.AngularVelocity.x) < 0.5f);
    CHECK(std::abs(world.Bodies().at(body).State.AngularVelocity.z) < 0.5f);
    CHECK(std::abs(world.Bodies().at(body).State.AngularVelocity.y) > 1.0f);
}

TEST_CASE("Solver islands ignore static bridges and merge through dynamic joints",
    "[PhysicsEngine][Island]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;
    const BodyID staticAnchor = world.CreateRigidBody(StaticBody());
    const BodyID a = world.CreateRigidBody(
        DynamicSphereBody(Vec3f{ -2.0f, 0.0f, 0.0f }, 0.25f));
    const BodyID b = world.CreateRigidBody(
        DynamicSphereBody(Vec3f{ 2.0f, 0.0f, 0.0f }, 0.25f));

    [[maybe_unused]] const JointID aJoint = world.CreateDistanceJoint(
        staticAnchor, a, Vec3f::Zero(), Vec3f{ -2.0f, 0.0f, 0.0f }, 2.0f);
    [[maybe_unused]] const JointID bJoint = world.CreateDistanceJoint(
        staticAnchor, b, Vec3f::Zero(), Vec3f{ 2.0f, 0.0f, 0.0f }, 2.0f);
    world.Step(1.0f / 60.0f);
    REQUIRE(world.SolverIslands().size() == 2u);

    [[maybe_unused]] const JointID bridge = world.CreateDistanceJoint(
        a, b,
        world.Bodies().at(a).State.Position,
        world.Bodies().at(b).State.Position,
        4.0f);
    world.Step(1.0f / 60.0f);
    CHECK(world.SolverIslands().size() == 1u);
}

TEST_CASE("Parallel island dispatch matches serial island results",
    "[PhysicsEngine][Island][Parallel]")
{
    auto configure = [](PhysicsWorld& world)
    {
        world.Gravity = Vec3f::Zero();
        world.Settings.EnableSleeping = false;
        world.Settings.ParallelIslandMinCount = 2;
        const BodyID anchor = world.CreateRigidBody(StaticBody());
        for (int i = 0; i < 4; ++i)
        {
            const float x = -3.0f + static_cast<float>(i) * 2.0f;
            RigidBodyDesc desc = DynamicSphereBody(Vec3f{ x, 0.0f, 0.0f }, 0.2f);
            desc.State.LinearVelocity = Vec3f{ 0.5f * static_cast<float>(i + 1), 1.0f, 0.0f };
            const BodyID body = world.CreateRigidBody(desc);
            [[maybe_unused]] const JointID joint = world.CreateDistanceJoint(
                anchor, body, Vec3f::Zero(), Vec3f{ x, 0.0f, 0.0f }, std::abs(x));
        }
    };

    PhysicsWorld serial;
    PhysicsWorld parallel;
    configure(serial);
    configure(parallel);
    serial.Settings.EnableParallelIslands = false;
    parallel.Settings.EnableParallelIslands = true;

    for (int step = 0; step < 90; ++step)
    {
        serial.Step(1.0f / 120.0f);
        parallel.Step(1.0f / 120.0f);
    }

    REQUIRE(serial.Bodies().size() == parallel.Bodies().size());
    for (std::size_t i = 0; i < serial.Bodies().size(); ++i)
    {
        CHECK((serial.Bodies()[i].State.Position - parallel.Bodies()[i].State.Position).Length() < 1.0e-4f);
        CHECK((serial.Bodies()[i].State.LinearVelocity - parallel.Bodies()[i].State.LinearVelocity).Length() < 1.0e-4f);
    }
}
'''
    first = s.find('\nTEST_CASE(')
    if first < 0:
        raise SystemExit('tests insertion marker not found')
    s = s[:first] + '\n' + tests + s[first:]
p.write_text(s)

# README milestone documentation.
p = Path('README.md')
s = p.read_text()
s = s.replace(
    'Sequential impulse solver with geometrically matched persistent multi-point manifolds and warm-started normal/friction impulses\n',
    'Sequential impulse solver with geometrically matched persistent multi-point manifolds and warm-started normal/friction impulses\nDistance, ball-socket, fixed, and hinge rigid-body joints with stable deletion-safe IDs\nDeterministic solver-island graph spanning contacts and joints, with optional parallel island worker dispatch\n',
    1)
s = s.replace(
    'ContactSolver        warm-started sequential impulses and position correction\nPhysicsDebug',
    'ContactSolver        warm-started sequential contact impulses\nJoint                distance/ball/fixed/hinge records plus velocity/position constraint solving\nSolverIsland         deterministic contact/joint graph partitioning and optional parallel dispatch\nPhysicsDebug',
    1)
s = s.replace('Island solver and parallel island dispatch\n', '', 1)
s = s.replace('Joints and articulated constraints\n', '', 1)
s = s.replace(
    'Near-term rigid-body work should add joints and island solving before larger\nphysics families are added.',
    'Near-term foundation work should add serialization/replay before larger\nphysics families are added.',
    1)

anchor = '## Contact Convention\n'
if anchor not in s:
    raise SystemExit('README contact convention marker not found')
joint_docs = r'''## Joints And Solver Islands

Joint creation uses world-space authoring inputs and stores body-local anchors so
constraints survive body motion without baking transient world coordinates:

```cpp
JointID hinge = world.CreateHingeJoint(
    chassisBody,
    doorBody,
    hingeWorldPosition,
    Vec3f::Up());
```

Connected bodies do not collide by default; pass `collideConnected=true` when a
mechanism intentionally needs both the constraint and collision response. Distance,
ball-socket, fixed, and hinge joints are solved in the same velocity/position
iterations as contacts.

Every step builds deterministic solver islands from blocking contacts and active
joints. Static and kinematic bodies are island boundaries rather than graph bridges,
so two crates touching the same floor remain independent islands. Parallel dispatch
is opt-in:

```cpp
world.Settings.EnableParallelIslands = true;
world.Settings.ParallelIslandMinCount = 4;
```

Each worker owns a disjoint set of dynamic bodies; contacts/joints inside one island
remain sequential and deterministic, while independent islands can execute in
parallel without cross-island body writes.

'''
s = s.replace(anchor, joint_docs + anchor, 1)
p.write_text(s)
