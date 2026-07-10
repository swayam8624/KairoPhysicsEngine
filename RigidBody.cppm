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
        BodyType Type = BodyType::Dynamic;
        MotionState State;
        MassProperties Mass = StaticMassProperties();
    };

    struct RigidBody final
    {
        BodyID ID = InvalidBodyID;
        bool Active = true;
        BodyType Type = BodyType::Dynamic;
        MotionState State;
        MassProperties Mass = StaticMassProperties();
        ForceAccumulation Forces;
        bool Sleeping = false;
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

        if (desc.Type == BodyType::Dynamic && desc.Mass.InverseMass <= 0.0f)
        {
            throw std::invalid_argument(
                "Dynamic rigid bodies require finite positive mass. "
                "Use BodyType::Static for immovable bodies.");
        }

        body.Mass = desc.Type == BodyType::Dynamic ? desc.Mass : StaticMassProperties();
        return body;
    }
}
