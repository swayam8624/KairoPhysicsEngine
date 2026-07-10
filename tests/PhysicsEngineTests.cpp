#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

import Kairo.Foundation.PhysicsEngine;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.Math.Vector;

using namespace kairo::foundation::physics;
using namespace kairo::foundation::math;

namespace
{
    RigidBodyDesc DynamicSphereBody(
        const Vec3f& position,
        float radius = 0.5f)
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Dynamic;
        desc.State.Position = position;
        desc.Mass = SphereMassProperties(radius, 1.0f);
        return desc;
    }

    RigidBodyDesc StaticBody(
        const Vec3f& position = Vec3f::Zero())
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Static;
        desc.State.Position = position;
        desc.Mass = StaticMassProperties();
        return desc;
    }
}

TEST_CASE("World creates bodies and colliders", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    const BodyID body =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 0.0f, 1.0f, 0.0f }));

    const ColliderID collider =
        world.AddCollider(body, SphereCollider{ 0.5f });

    REQUIRE(body == 0);
    REQUIRE(collider == 0);
    REQUIRE(world.Bodies().size() == 1);
    REQUIRE(world.Colliders().size() == 1);
}

TEST_CASE("Broadphase pairs overlapping finite colliders and planes", "[PhysicsEngine][Broadphase]")
{
    std::vector<RigidBody> bodies;
    bodies.push_back(MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.5f, 0.0f })));
    bodies.push_back(MakeRigidBody(1, DynamicSphereBody(Vec3f{ 0.6f, 0.5f, 0.0f })));
    bodies.push_back(MakeRigidBody(2, StaticBody()));

    std::vector<Collider> colliders;
    colliders.push_back(MakeCollider(0, 0, SphereCollider{ 0.5f }));
    colliders.push_back(MakeCollider(1, 1, SphereCollider{ 0.5f }));
    colliders.push_back(MakeCollider(2, 2, PlaneCollider{ Vec3f::Up(), 0.0f }));

    const std::vector<BroadphasePair> pairs =
        ComputeBroadphasePairs(bodies, colliders);

    REQUIRE(pairs.size() == 3);
}

TEST_CASE("Narrowphase creates sphere and plane contacts", "[PhysicsEngine][Narrowphase]")
{
    const RigidBody sphereBody =
        MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.4f, 0.0f }));

    const RigidBody planeBody =
        MakeRigidBody(1, StaticBody());

    const Collider sphere =
        MakeCollider(0, 0, SphereCollider{ 0.5f });

    const Collider plane =
        MakeCollider(1, 1, PlaneCollider{ Vec3f::Up(), 0.0f });

    const auto contact =
        CollidePair(sphereBody, sphere, planeBody, plane);

    REQUIRE(contact.has_value());
    REQUIRE(contact->Points.size() == 1);
    REQUIRE(contact->Points[0].PenetrationDepth == Catch::Approx(0.1f));
    REQUIRE(contact->Points[0].Normal.y == Catch::Approx(-1.0f));
}

TEST_CASE("Contact solver reverses closing normal velocity", "[PhysicsEngine][Solver]")
{
    std::vector<RigidBody> bodies;
    bodies.push_back(MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.4f, 0.0f })));
    bodies.push_back(MakeRigidBody(1, StaticBody()));
    bodies[0].State.LinearVelocity = Vec3f{ 0.0f, -2.0f, 0.0f };

    std::vector<Collider> colliders;
    PhysicsMaterial material;
    material.Restitution = 0.0f;
    colliders.push_back(MakeCollider(0, 0, SphereCollider{ 0.5f }, material));
    colliders.push_back(MakeCollider(1, 1, PlaneCollider{ Vec3f::Up(), 0.0f }, material));

    ContactManifold manifold =
        MakeContactManifold(0, 1);
    manifold.Points.push_back(
        MakeContactPoint(
            Vec3f{ 0.0f, 0.0f, 0.0f },
            Vec3f{ 0.0f, -1.0f, 0.0f },
            0.1f));

    PhysicsStepSettings settings;
    settings.VelocityIterations = 4;
    settings.Baumgarte = 0.0f;

    SolveContacts(bodies, colliders, { manifold }, settings, 1.0f / 60.0f);

    REQUIRE(bodies[0].State.LinearVelocity.y >= Catch::Approx(0.0f).margin(1.0e-4f));
}

TEST_CASE("PhysicsWorld settles falling sphere against plane", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    world.Settings.VelocityIterations = 10;
    world.Settings.Baumgarte = 0.1f;

    const BodyID sphere =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 0.0f, 1.2f, 0.0f }));

    const BodyID floor =
        world.CreateRigidBody(StaticBody());

    [[maybe_unused]] const ColliderID sphereCollider =
        world.AddCollider(sphere, SphereCollider{ 0.5f });

    [[maybe_unused]] const ColliderID floorCollider =
        world.AddCollider(floor, PlaneCollider{ Vec3f::Up(), 0.0f });

    for (int i = 0; i < 90; ++i)
    {
        world.Step(1.0f / 60.0f);
    }

    REQUIRE(world.Bodies()[sphere].State.Position.y >= 0.48f);
    REQUIRE(!world.Contacts().empty());
    REQUIRE(!world.DebugContacts().empty());
    REQUIRE(world.DebugAABBs().size() == 1);
}

TEST_CASE("AABB contacts are detected", "[PhysicsEngine][Narrowphase]")
{
    const RigidBody a =
        MakeRigidBody(0, StaticBody(Vec3f::Zero()));

    const RigidBody b =
        MakeRigidBody(1, StaticBody(Vec3f{ 0.75f, 0.0f, 0.0f }));

    const Collider ca =
        MakeCollider(0, 0, AABBCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });

    const Collider cb =
        MakeCollider(1, 1, AABBCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });

    const auto contact =
        CollidePair(a, ca, b, cb);

    REQUIRE(contact.has_value());
    REQUIRE(contact->Points[0].PenetrationDepth == Catch::Approx(0.25f));
}

TEST_CASE("Invalid world inputs throw", "[PhysicsEngine][Validation]")
{
    PhysicsWorld world;
    REQUIRE_THROWS_AS(world.Step(0.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(world.AddCollider(42, SphereCollider{ 0.5f }), std::out_of_range);

    const BodyID body =
        world.CreateRigidBody(DynamicSphereBody(Vec3f::Zero()));

    REQUIRE_THROWS_AS(world.AddCollider(body, SphereCollider{ -1.0f }), std::invalid_argument);
}
