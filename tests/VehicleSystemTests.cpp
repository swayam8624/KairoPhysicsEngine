#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

import Kairo.Foundation.PhysicsEngine;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.Math.Vector;

using namespace kairo::foundation::physics;
using namespace kairo::foundation::math;

namespace
{
    RigidBodyDesc StaticGroundBody()
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Static;
        desc.Mass = StaticMassProperties();
        return desc;
    }

    RigidBodyDesc VehicleChassisBody(
        const Vec3f& position = Vec3f{ 0.0f, 0.87f, 0.0f },
        const Vec3f& velocity = Vec3f::Zero())
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Dynamic;
        desc.State.Position = position;
        desc.State.LinearVelocity = velocity;
        desc.Mass = BoxMassProperties(Vec3f{ 0.80f, 0.25f, 1.35f }, 180.0f);
        desc.LinearDamping = 0.02f;
        desc.AngularDamping = 0.08f;
        desc.AllowSleeping = false;
        return desc;
    }

    VehicleWheelDesc MakeWheel(float x, float z, bool steer, bool drive,
        bool handbrake = false)
    {
        VehicleWheelDesc wheel;
        wheel.LocalMount = Vec3f{ x, -0.20f, z };
        wheel.LocalSuspensionDirection = Vec3f{ 0.0f, -1.0f, 0.0f };
        wheel.LocalForward = Vec3f::UnitZ();
        wheel.Radius = 0.30f;
        wheel.RestLength = 0.40f;
        wheel.MaxCompression = 0.18f;
        wheel.MaxDroop = 0.15f;
        wheel.SpringStiffness = 32'000.0f;
        wheel.CompressionDamping = 4'000.0f;
        wheel.ReboundDamping = 5'000.0f;
        wheel.LateralStiffness = 2'400.0f;
        wheel.LongitudinalStiffness = 90.0f;
        wheel.FrictionCoefficient = 1.20f;
        wheel.DriveFactor = drive ? 1.0f : 0.0f;
        wheel.BrakeFactor = 1.0f;
        wheel.HandbrakeFactor = handbrake ? 1.0f : 0.0f;
        wheel.SteerFactor = steer ? 1.0f : 0.0f;
        wheel.GroundMask = CollisionLayer::StaticWorld |
            CollisionLayer::DynamicWorld | CollisionLayer::Default;
        return wheel;
    }

    VehicleDesc FourWheelVehicle(BodyID chassis)
    {
        VehicleDesc desc;
        desc.Chassis = chassis;
        desc.Wheels = {
            MakeWheel(-0.68f,  0.95f, true,  false),
            MakeWheel( 0.68f,  0.95f, true,  false),
            MakeWheel(-0.68f, -0.95f, false, true, true),
            MakeWheel( 0.68f, -0.95f, false, true, true)
        };
        desc.MaxSteerAngleRadians = 0.52f;
        desc.MaxDriveForce = 8'500.0f;
        desc.MaxBrakeForce = 13'000.0f;
        desc.MaxHandbrakeForce = 17'000.0f;
        desc.AerodynamicDrag = 0.25f;
        desc.DownforceCoefficient = 0.0f;
        desc.RollingResistance = 20.0f;
        return desc;
    }

    struct VehicleFixture final
    {
        PhysicsWorld World;
        VehicleSystem Vehicles;
        BodyID Ground = InvalidBodyID;
        BodyID Chassis = InvalidBodyID;
        VehicleID Vehicle = InvalidVehicleID;

        explicit VehicleFixture(Vec3f initialVelocity = Vec3f::Zero())
        {
            World.Settings.EnableSleeping = false;
            Ground = World.CreateRigidBody(StaticGroundBody());
            const ColliderID groundCollider = World.AddCollider(
                Ground, PlaneCollider{ Vec3f::Up(), 0.0f });
            World.SetCollisionFilter(groundCollider,
                CollisionLayer::StaticWorld, CollisionLayer::All);

            // The chassis mass is about 389 kg. With four 32 kN/m springs,
            // static load needs about 0.03 m compression, placing the chassis
            // at roughly y=0.87. Starting at y=0.75 preloads 0.15 m per wheel
            // and launches the fixture upward before the drive assertion runs.
            Chassis = World.CreateRigidBody(
                VehicleChassisBody(Vec3f{ 0.0f, 0.87f, 0.0f }, initialVelocity));
            const ColliderID chassisCollider = World.AddCollider(
                Chassis, BoxCollider{ Vec3f{ 0.80f, 0.25f, 1.35f } });
            World.SetCollisionFilter(chassisCollider,
                CollisionLayer::DynamicWorld, CollisionLayer::All);
            Vehicle = Vehicles.CreateVehicle(World, FourWheelVehicle(Chassis));
        }
    };
}

TEST_CASE("Raycast vehicle suspension detects ground and carries chassis load",
    "[PhysicsEngine][Vehicle][Suspension]")
{
    VehicleFixture fixture;
    fixture.Vehicles.Step(fixture.World, 1.0f / 120.0f);

    const auto& wheels = fixture.Vehicles.Wheels(fixture.Vehicle);
    REQUIRE(wheels.size() == 4u);
    CHECK(fixture.Vehicles.State(fixture.Vehicle).GroundedWheels == 4u);
    for (const VehicleWheelState& wheel : wheels)
    {
        CHECK(wheel.Grounded);
        CHECK(wheel.GroundBody == fixture.Ground);
        CHECK(wheel.SuspensionLength < 0.40f);
        CHECK(wheel.SuspensionCompression > 0.0f);
        CHECK(wheel.SuspensionForce > 0.0f);
        CHECK(wheel.ContactNormal.y > 0.99f);
    }
}

TEST_CASE("Driven wheels accelerate a grounded chassis in authored forward direction",
    "[PhysicsEngine][Vehicle][Drive]")
{
    VehicleFixture fixture;
    fixture.Vehicles.SetControls(fixture.Vehicle,
        VehicleControls{ .Throttle = 1.0f });

    constexpr float dt = 1.0f / 120.0f;
    std::size_t groundedFrames = 0u;
    std::size_t maximumGroundedWheels = 0u;
    for (int step = 0; step < 90; ++step)
    {
        fixture.Vehicles.Step(fixture.World, dt);
        const std::size_t grounded =
            fixture.Vehicles.State(fixture.Vehicle).GroundedWheels;
        if (grounded >= 2u) ++groundedFrames;
        maximumGroundedWheels = std::max(maximumGroundedWheels, grounded);
        fixture.World.Step(dt);
    }

    const RigidBody& chassis = fixture.World.Bodies().at(fixture.Chassis);
    const auto& state = fixture.Vehicles.State(fixture.Vehicle);
    CAPTURE(groundedFrames, maximumGroundedWheels, state.GroundedWheels,
        chassis.State.Position.x, chassis.State.Position.y, chassis.State.Position.z,
        chassis.State.LinearVelocity.x, chassis.State.LinearVelocity.y,
        chassis.State.LinearVelocity.z, chassis.State.Rotation.x,
        chassis.State.Rotation.y, chassis.State.Rotation.z,
        chassis.State.Rotation.w);
    CHECK(chassis.State.LinearVelocity.z > 0.25f);
    CHECK(chassis.State.Position.z > 0.05f);
    CHECK(state.GroundedWheels >= 2u);
}

TEST_CASE("Steered wheels generate lateral tire response from forward motion",
    "[PhysicsEngine][Vehicle][Steering]")
{
    VehicleFixture fixture(Vec3f{ 0.0f, 0.0f, 8.0f });
    fixture.Vehicles.SetControls(fixture.Vehicle,
        VehicleControls{ .Steering = 0.75f });
    fixture.Vehicles.Step(fixture.World, 1.0f / 120.0f);

    const auto& wheels = fixture.Vehicles.Wheels(fixture.Vehicle);
    REQUIRE(wheels.size() == 4u);
    CHECK(std::abs(wheels[0].SteeringAngle) > 0.1f);
    CHECK(std::abs(wheels[1].SteeringAngle) > 0.1f);
    CHECK(wheels[2].SteeringAngle == Catch::Approx(0.0f));
    CHECK(wheels[3].SteeringAngle == Catch::Approx(0.0f));
    CHECK(std::abs(wheels[0].LateralSpeed) > 0.1f);
    CHECK(std::abs(wheels[0].LateralForce) > 1.0f);
}

TEST_CASE("Vehicle tire forces never exceed the suspension friction circle",
    "[PhysicsEngine][Vehicle][Tire]")
{
    VehicleFixture fixture(Vec3f{ 3.0f, 0.0f, 10.0f });
    fixture.Vehicles.SetControls(fixture.Vehicle,
        VehicleControls{
            .Throttle = 1.0f,
            .Steering = -1.0f,
            .Brake = 0.25f,
            .Handbrake = 0.35f
        });
    fixture.Vehicles.Step(fixture.World, 1.0f / 120.0f);

    const VehicleDesc& desc = fixture.Vehicles.Descriptor(fixture.Vehicle);
    const auto& states = fixture.Vehicles.Wheels(fixture.Vehicle);
    REQUIRE(states.size() == desc.Wheels.size());
    for (std::size_t index = 0u; index < states.size(); ++index)
    {
        if (!states[index].Grounded) continue;
        const float magnitude = std::hypot(
            states[index].LongitudinalForce,
            states[index].LateralForce);
        const float limit = desc.Wheels[index].FrictionCoefficient *
            states[index].SuspensionForce;
        CHECK(magnitude <= limit + 1.0e-3f);
    }
}

TEST_CASE("Vehicle system validates chassis and normalized control contracts",
    "[PhysicsEngine][Vehicle][Validation]")
{
    PhysicsWorld world;
    VehicleSystem vehicles;
    VehicleDesc invalid;
    invalid.Chassis = 99u;
    invalid.Wheels.push_back(MakeWheel(0.0f, 0.0f, false, true));
    REQUIRE_THROWS_AS(vehicles.CreateVehicle(world, invalid), std::out_of_range);

    const BodyID staticBody = world.CreateRigidBody(StaticGroundBody());
    invalid.Chassis = staticBody;
    REQUIRE_THROWS_AS(vehicles.CreateVehicle(world, invalid), std::invalid_argument);

    const BodyID chassis = world.CreateRigidBody(VehicleChassisBody());
    const VehicleID id = vehicles.CreateVehicle(world, FourWheelVehicle(chassis));
    REQUIRE_THROWS_AS(vehicles.SetControls(id,
        VehicleControls{ .Throttle = 1.1f }), std::invalid_argument);
    vehicles.RemoveVehicle(id);
    CHECK_FALSE(vehicles.IsValidVehicle(id));
    REQUIRE_THROWS_AS(vehicles.State(id), std::out_of_range);
}
