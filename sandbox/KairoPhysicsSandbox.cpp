#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>

import Kairo.Foundation.PhysicsEngine;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.Math.Vector;

using namespace kairo::foundation::math;
using namespace kairo::foundation::physics;

namespace
{
    RigidBodyDesc DynamicSphere(
        const Vec3f& position,
        float radius = 0.5f)
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Dynamic;
        desc.State.Position = position;
        desc.Mass = SphereMassProperties(radius, 1.0f);
        desc.LinearDamping = 0.02f;
        desc.AngularDamping = 0.02f;
        return desc;
    }

    RigidBodyDesc StaticBodyAt(
        const Vec3f& position = Vec3f::Zero())
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Static;
        desc.State.Position = position;
        return desc;
    }

    void PrintHeader(
        std::string_view name)
    {
        std::cout
            << "# KairoPhysicsSandbox scenario=" << name << '\n'
            << "# columns: step,time,pos.x,pos.y,pos.z,vel.x,vel.y,vel.z,contacts,events\n";
    }

    void PrintBodyRow(
        int step,
        float time,
        const PhysicsWorld& world,
        BodyID body)
    {
        const MotionState& state =
            world.Bodies().at(body).State;

        std::cout
            << step << ','
            << std::fixed << std::setprecision(4) << time << ','
            << state.Position.x << ','
            << state.Position.y << ','
            << state.Position.z << ','
            << state.LinearVelocity.x << ','
            << state.LinearVelocity.y << ','
            << state.LinearVelocity.z << ','
            << world.Contacts().size() << ','
            << world.ContactEvents().size() << '\n';
    }

    int RunFallingSphere()
    {
        PhysicsWorld world;
        world.Settings.VelocityIterations = 16;
        world.Settings.PositionIterations = 6;

        const BodyID sphere =
            world.CreateRigidBody(DynamicSphere(Vec3f{ 0.0f, 3.0f, 0.0f }));

        const BodyID floor =
            world.CreateRigidBody(StaticBodyAt());

        [[maybe_unused]] const ColliderID sphereCollider =
            world.AddCollider(sphere, SphereCollider{ 0.5f });

        [[maybe_unused]] const ColliderID floorCollider =
            world.AddCollider(floor, PlaneCollider{ Vec3f::Up(), 0.0f });

        constexpr float fixedDt =
            1.0f / 60.0f;

        PrintHeader("falling-sphere");
        for (int step = 0; step <= 180; ++step)
        {
            PrintBodyRow(step, static_cast<float>(step) * fixedDt, world, sphere);
            world.Step(fixedDt);
        }

        return 0;
    }

    int RunTrigger()
    {
        PhysicsWorld world;
        world.Gravity = Vec3f::Zero();
        world.Settings.EnableSleeping = false;

        RigidBodyDesc mover =
            DynamicSphere(Vec3f{ -2.0f, 0.0f, 0.0f });
        mover.EnableGravity = false;
        mover.State.LinearVelocity = Vec3f{ 2.0f, 0.0f, 0.0f };

        const BodyID sphere =
            world.CreateRigidBody(mover);

        const BodyID trigger =
            world.CreateRigidBody(StaticBodyAt(Vec3f::Zero()));

        [[maybe_unused]] const ColliderID sphereCollider =
            world.AddCollider(sphere, SphereCollider{ 0.4f });

        const ColliderID triggerCollider =
            world.AddCollider(trigger, BoxCollider{ Vec3f{ 0.25f, 1.0f, 1.0f } });
        world.SetColliderTrigger(triggerCollider, true);

        constexpr float fixedDt =
            1.0f / 30.0f;

        PrintHeader("trigger-volume");
        for (int step = 0; step <= 90; ++step)
        {
            PrintBodyRow(step, static_cast<float>(step) * fixedDt, world, sphere);
            for (const PhysicsContactEvent& event : world.ContactEvents())
            {
                const char* type =
                    event.Type == PhysicsContactEventType::Begin
                        ? "begin"
                        : (event.Type == PhysicsContactEventType::Stay ? "stay" : "end");

                std::cout
                    << "# event step=" << step
                    << " type=" << type
                    << " trigger=" << (event.IsTrigger ? "true" : "false")
                    << " colliderA=" << event.ColliderA
                    << " colliderB=" << event.ColliderB
                    << '\n';
            }
            world.Step(fixedDt);
        }

        return 0;
    }

    int RunRotatedBox()
    {
        PhysicsWorld world;
        world.Settings.VelocityIterations = 18;
        world.Settings.PositionIterations = 8;

        RigidBodyDesc box =
            DynamicSphere(Vec3f{ 0.0f, 2.0f, 0.0f }, 0.5f);
        box.Mass = BoxMassProperties(Vec3f{ 0.5f, 0.5f, 0.5f }, 1.0f);
        box.State.Rotation = RotationAroundZ(0.35f);

        const BodyID dynamicBox =
            world.CreateRigidBody(box);

        const BodyID floor =
            world.CreateRigidBody(StaticBodyAt());

        [[maybe_unused]] const ColliderID boxCollider =
            world.AddCollider(
            dynamicBox,
            BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } },
            {},
            Vec3f::Zero(),
            RotationAroundZ(0.25f));

        [[maybe_unused]] const ColliderID floorCollider =
            world.AddCollider(floor, PlaneCollider{ Vec3f::Up(), 0.0f });

        constexpr float fixedDt =
            1.0f / 60.0f;

        PrintHeader("rotated-box");
        for (int step = 0; step <= 180; ++step)
        {
            PrintBodyRow(step, static_cast<float>(step) * fixedDt, world, dynamicBox);
            world.Step(fixedDt);
        }

        return 0;
    }

    void PrintUsage(
        const char* executable)
    {
        std::cerr
            << "Usage: " << executable << " [falling-sphere|trigger|rotated-box]\n";
    }
}

int main(
    int argc,
    char** argv)
{
    const std::string scenario =
        argc > 1 ? std::string(argv[1]) : std::string("falling-sphere");

    if (scenario == "falling-sphere")
    {
        return RunFallingSphere();
    }

    if (scenario == "trigger")
    {
        return RunTrigger();
    }

    if (scenario == "rotated-box")
    {
        return RunRotatedBox();
    }

    PrintUsage(argv[0]);
    return 2;
}
