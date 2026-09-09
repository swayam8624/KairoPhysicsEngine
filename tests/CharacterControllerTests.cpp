#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <stdexcept>

import Kairo.Foundation.PhysicsEngine;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.Math.Vector;

using namespace kairo::foundation::physics;
using namespace kairo::foundation::math;

namespace
{
    [[nodiscard]] RigidBodyDesc StaticBody(const Vec3f& position = Vec3f::Zero())
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Static;
        desc.State.Position = position;
        desc.Mass = StaticMassProperties();
        return desc;
    }

    [[nodiscard]] PhysicsWorld WorldWithFloor()
    {
        PhysicsWorld world;
        world.Gravity = Vec3f::Zero();
        const BodyID floor = world.CreateRigidBody(StaticBody());
        const ColliderID collider = world.AddCollider(
            floor,
            PlaneCollider{ Vec3f::Up(), 0.0f });
        world.SetColliderCollisionLayer(collider, CollisionLayer::StaticWorld);
        return world;
    }

    [[nodiscard]] CharacterControllerConfig TestConfig()
    {
        CharacterControllerConfig config;
        config.Radius = 0.30f;
        config.Height = 1.80f;
        config.SkinWidth = 0.02f;
        config.GroundProbeDistance = 0.15f;
        config.MaxSlideIterations = 6u;
        config.SweepSamples = 5u;
        config.CollisionMask = CollisionLayer::StaticWorld;
        return config;
    }
}

TEST_CASE("character controller probes and preserves a stable floor skin")
{
    PhysicsWorld world = WorldWithFloor();
    KinematicCharacterController controller(TestConfig(),
        Vec3f{ 0.0f, 0.92f, 0.0f });

    const CharacterGroundState ground = controller.ProbeGround(world);
    REQUIRE(ground.Grounded);
    CHECK(ground.Normal.y == Catch::Approx(1.0f).margin(1.0e-4f));
    CHECK(ground.Distance == Catch::Approx(0.02f).margin(2.0e-3f));

    const auto move = controller.Move(world, Vec3f{ 1.0f, 0.0f, 0.0f });
    CHECK(move.Ground.Grounded);
    CHECK(controller.Position().x == Catch::Approx(1.0f).margin(1.0e-3f));
    CHECK(controller.Position().y == Catch::Approx(0.92f).margin(2.0e-3f));
}

TEST_CASE("character controller stops at walls and retains skin width")
{
    PhysicsWorld world = WorldWithFloor();
    const BodyID wall = world.CreateRigidBody(StaticBody(Vec3f{ 2.0f, 1.0f, 0.0f }));
    const ColliderID wallCollider = world.AddCollider(
        wall,
        AABBCollider{ Vec3f{ 0.25f, 1.0f, 2.0f } });
    world.SetColliderCollisionLayer(wallCollider, CollisionLayer::StaticWorld);

    KinematicCharacterController controller(TestConfig(),
        Vec3f{ 0.0f, 0.92f, 0.0f });
    const auto move = controller.Move(world, Vec3f{ 4.0f, 0.0f, 0.0f });

    CHECK(move.HitWall);
    CHECK(controller.Position().x == Catch::Approx(1.43f).margin(0.03f));
    CHECK(move.RemainingDisplacement.Length() <= 1.0e-3f);
    CHECK(move.Ground.Grounded);
}

TEST_CASE("character controller slides tangentially instead of sticking to a wall")
{
    PhysicsWorld world = WorldWithFloor();
    const BodyID wall = world.CreateRigidBody(StaticBody(Vec3f{ 2.0f, 1.0f, 0.0f }));
    const ColliderID wallCollider = world.AddCollider(
        wall,
        AABBCollider{ Vec3f{ 0.25f, 1.0f, 5.0f } });
    world.SetColliderCollisionLayer(wallCollider, CollisionLayer::StaticWorld);

    KinematicCharacterController controller(TestConfig(),
        Vec3f{ 0.0f, 0.92f, -1.0f });
    const auto move = controller.Move(world, Vec3f{ 4.0f, 0.0f, 2.0f });

    CHECK(move.HitWall);
    CHECK(controller.Position().x < 1.48f);
    CHECK(controller.Position().z > 0.5f);
    CHECK(move.AppliedDisplacement.z > 1.5f);
}

TEST_CASE("character controller ground snap settles small downward gaps")
{
    PhysicsWorld world = WorldWithFloor();
    KinematicCharacterController controller(TestConfig(),
        Vec3f{ 0.0f, 1.00f, 0.0f });

    const auto move = controller.Move(world, Vec3f{ 0.25f, 0.0f, 0.0f });
    REQUIRE(move.Ground.Grounded);
    CHECK(controller.Position().y == Catch::Approx(0.92f).margin(0.01f));
    CHECK(move.AppliedDisplacement.y < -0.06f);
}

TEST_CASE("character controller does not ground-snap an upward jump")
{
    PhysicsWorld world = WorldWithFloor();
    KinematicCharacterController controller(TestConfig(),
        Vec3f{ 0.0f, 0.92f, 0.0f });

    const auto move = controller.Move(world, Vec3f{ 0.0f, 0.50f, 0.0f });
    CHECK(controller.Position().y == Catch::Approx(1.42f).margin(1.0e-3f));
    CHECK_FALSE(move.Ground.Grounded);
}

TEST_CASE("character controller reports ceiling contacts")
{
    PhysicsWorld world = WorldWithFloor();
    const BodyID ceiling = world.CreateRigidBody(StaticBody(Vec3f{ 0.0f, 2.30f, 0.0f }));
    const ColliderID ceilingCollider = world.AddCollider(
        ceiling,
        AABBCollider{ Vec3f{ 2.0f, 0.10f, 2.0f } });
    world.SetColliderCollisionLayer(ceilingCollider, CollisionLayer::StaticWorld);

    KinematicCharacterController controller(TestConfig(),
        Vec3f{ 0.0f, 0.92f, 0.0f });
    const auto move = controller.Move(world, Vec3f{ 0.0f, 2.0f, 0.0f });

    CHECK(move.HitCeiling);
    CHECK(controller.Position().y < 1.40f);
}

TEST_CASE("character controller validates unsafe shape settings")
{
    CharacterControllerConfig config = TestConfig();
    config.Height = 0.40f;
    CHECK_THROWS_AS(
        KinematicCharacterController(config, Vec3f::Zero()),
        std::invalid_argument);

    config = TestConfig();
    config.SkinWidth = config.Radius;
    CHECK_THROWS_AS(
        KinematicCharacterController(config, Vec3f::Zero()),
        std::invalid_argument);
}
