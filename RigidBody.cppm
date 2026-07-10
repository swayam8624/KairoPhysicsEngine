module;

#include <stdexcept>
#include <vector>

export module Kairo.Foundation.PhysicsEngine.RigidBody;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.PhysicsMath;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    struct RigidBodyDesc final
    {
        BodyType Type = BodyType::Static;
        MotionState State;
        MassProperties Mass = StaticMassProperties();
        bool EnableGravity = true;
        float GravityScale = 1.0f;
        float LinearDamping = 0.0f;
        float AngularDamping = 0.0f;
        float MaxLinearSpeed = 1000.0f;
        float MaxAngularSpeed = 1000.0f;
        bool AllowSleeping = true;
    };

    struct RigidBody final
    {
        BodyID ID = InvalidBodyID;
        bool Active = true;
        BodyType Type = BodyType::Dynamic;
        MotionState State;
        MassProperties Mass = StaticMassProperties();
        ForceAccumulation Forces;
        bool EnableGravity = true;
        float GravityScale = 1.0f;
        float LinearDamping = 0.0f;
        float AngularDamping = 0.0f;
        float MaxLinearSpeed = 1000.0f;
        float MaxAngularSpeed = 1000.0f;
        bool AllowSleeping = true;
        bool Sleeping = false;
        float SleepTimer = 0.0f;
    };

    /// Input: body.
    /// Output: true when impulses/forces should modify velocity.
    /// Task: keep static and kinematic bodies out of solver velocity mutation.
    [[nodiscard]]
    inline bool IsDynamic(
        const RigidBody& body) noexcept
    {
        return body.Active &&
            body.Type == BodyType::Dynamic &&
            body.Mass.InverseMass > 0.0f &&
            !body.Sleeping;
    }

    /// Input: body.
    /// Output: true when the record is still owned by the world.
    /// Task: preserve stable vector-index ids while allowing deletion-safe
    /// handles. Removed bodies stay in storage but are ignored by simulation.
    [[nodiscard]]
    inline bool IsActiveBody(
        const RigidBody& body) noexcept
    {
        return body.Active &&
            body.ID != InvalidBodyID;
    }

    /// Input: body.
    /// Output: true when the body is dynamic even if it is currently sleeping.
    /// Task: separate "can be woken and simulated" from `IsDynamic`, which
    /// intentionally excludes sleeping bodies from impulse mutation.
    [[nodiscard]]
    inline bool IsDynamicBodyType(
        const RigidBody& body) noexcept
    {
        return body.Active &&
            body.Type == BodyType::Dynamic &&
            body.Mass.InverseMass > 0.0f;
    }

    /// Input: body.
    /// Output: true when the body should advance by authored velocity only.
    /// Task: make kinematic objects explicit: solver impulses and forces do not
    /// move them, but `PhysicsWorld::Step` advances their motion state.
    [[nodiscard]]
    inline bool IsKinematic(
        const RigidBody& body) noexcept
    {
        return body.Active &&
            body.Type == BodyType::Kinematic;
    }

    /// Input: body.
    /// Output: true if the body has queued force or torque.
    /// Task: wake sleeping bodies when user code applies force through the
    /// accumulator before a step.
    [[nodiscard]]
    inline bool HasAccumulatedForce(
        const RigidBody& body) noexcept
    {
        return body.Forces.Force.LengthSquared() > 1.0e-12f ||
            body.Forces.Torque.LengthSquared() > 1.0e-12f;
    }

    /// Input: body.
    /// Output: body marked awake and ready for dynamic integration.
    /// Task: centralize wake behavior so forces, impulses, and contacts reset
    /// sleep state consistently.
    inline void WakeRigidBody(
        RigidBody& body) noexcept
    {
        body.Sleeping = false;
        body.SleepTimer = 0.0f;
    }

    /// Input: body.
    /// Output: world-space inverse inertia tensor.
    /// Task: expose the rotational mass used by contact impulses.
    [[nodiscard]]
    inline Matrix3f WorldInverseInertia(
        const RigidBody& body)
    {
        if (!IsDynamic(body))
        {
            return Matrix3f::Zero();
        }

        return InverseInertiaWorld(
            body.Mass.LocalInverseInertiaTensor,
            body.State.Rotation);
    }

    /// Input: body descriptor and generated id.
    /// Output: validated rigid body record.
    /// Task: centralize body construction so world storage stays simple.
    [[nodiscard]]
    inline RigidBody MakeRigidBody(
        BodyID id,
        const RigidBodyDesc& desc)
    {
        RigidBody body;
        body.ID = id;
        body.Active = true;
        body.Type = desc.Type;
        body.State = desc.State;

        RequireFinite(body.State.Position, "RigidBodyDesc.State.Position");
        RequireFinite(body.State.LinearVelocity, "RigidBodyDesc.State.LinearVelocity");
        RequireFinite(body.State.AngularVelocity, "RigidBodyDesc.State.AngularVelocity");
        RequireFinite(desc.GravityScale, "RigidBodyDesc.GravityScale");
        RequireNonNegative(desc.LinearDamping, "RigidBodyDesc.LinearDamping");
        RequireNonNegative(desc.AngularDamping, "RigidBodyDesc.AngularDamping");
        RequirePositive(desc.MaxLinearSpeed, "RigidBodyDesc.MaxLinearSpeed");
        RequirePositive(desc.MaxAngularSpeed, "RigidBodyDesc.MaxAngularSpeed");

        if (desc.Type == BodyType::Dynamic && desc.Mass.InverseMass <= 0.0f)
        {
            throw std::invalid_argument(
                "Dynamic rigid bodies require finite positive mass. "
                "Use BodyType::Static for immovable bodies.");
        }

        body.Mass = desc.Type == BodyType::Dynamic ? desc.Mass : StaticMassProperties();
        body.EnableGravity = desc.EnableGravity;
        body.GravityScale = desc.GravityScale;
        body.LinearDamping = desc.LinearDamping;
        body.AngularDamping = desc.AngularDamping;
        body.MaxLinearSpeed = desc.MaxLinearSpeed;
        body.MaxAngularSpeed = desc.MaxAngularSpeed;
        body.AllowSleeping = desc.AllowSleeping;
        body.Sleeping = false;
        body.SleepTimer = 0.0f;
        return body;
    }
}
