module;

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
