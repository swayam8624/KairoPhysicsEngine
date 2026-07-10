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

TEST_CASE("World removes colliders and destroys bodies without reusing ids", "[PhysicsEngine][World]")
{
    PhysicsWorld world;

    const BodyID bodyA =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 0.0f, 0.5f, 0.0f }));

    const BodyID bodyB =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 0.75f, 0.5f, 0.0f }));

    const ColliderID colliderA =
        world.AddCollider(bodyA, SphereCollider{ 0.5f });

    const ColliderID colliderB =
        world.AddCollider(bodyB, SphereCollider{ 0.5f });

    world.Step(1.0f / 60.0f);
    REQUIRE(world.IsValidBody(bodyA));
    REQUIRE(world.IsValidCollider(colliderA));
    REQUIRE(!world.BroadphasePairs().empty());
    REQUIRE(!world.Contacts().empty());

    world.RemoveCollider(colliderA);
    REQUIRE(!world.IsValidCollider(colliderA));
    REQUIRE(world.IsValidCollider(colliderB));
    REQUIRE(world.BroadphasePairs().empty());
    REQUIRE(world.Contacts().empty());

    world.DestroyRigidBody(bodyB);
    REQUIRE(!world.IsValidBody(bodyB));
    REQUIRE(!world.IsValidCollider(colliderB));
    REQUIRE_THROWS_AS(world.AddCollider(bodyB, SphereCollider{ 0.5f }), std::out_of_range);

    const BodyID bodyC =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 2.0f, 0.5f, 0.0f }));

    REQUIRE(bodyC == 2);
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

TEST_CASE("Persistent broadphase updates moved colliders and excludes same-body pairs", "[PhysicsEngine][Broadphase]")
{
    std::vector<RigidBody> bodies;
    bodies.push_back(MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.5f, 0.0f })));
    bodies.push_back(MakeRigidBody(1, DynamicSphereBody(Vec3f{ 5.0f, 0.5f, 0.0f })));

    std::vector<Collider> colliders;
    colliders.push_back(MakeCollider(0, 0, SphereCollider{ 0.5f }));
    colliders.push_back(MakeCollider(1, 0, SphereCollider{ 0.5f }, {}, Vec3f{ 0.1f, 0.0f, 0.0f }));
    colliders.push_back(MakeCollider(2, 1, SphereCollider{ 0.5f }));

    BroadphaseWorld broadphase;
    broadphase.Sync(bodies, colliders);
    REQUIRE(broadphase.ComputePairs(bodies, colliders).empty());

    bodies[1].State.Position = Vec3f{ 0.7f, 0.5f, 0.0f };
    broadphase.Sync(bodies, colliders);

    const std::vector<BroadphasePair> pairs =
        broadphase.ComputePairs(bodies, colliders);

    REQUIRE(pairs.size() == 2);
    REQUIRE(pairs[0].A == 0);
    REQUIRE(pairs[0].B == 2);
    REQUIRE(pairs[1].A == 1);
    REQUIRE(pairs[1].B == 2);
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
        MakeContactManifold(0, 1, 0, 1);
    manifold.Points.push_back(
        MakeContactPoint(
            Vec3f{ 0.0f, 0.0f, 0.0f },
            Vec3f{ 0.0f, -1.0f, 0.0f },
            0.1f));

    PhysicsStepSettings settings;
    settings.VelocityIterations = 4;
    settings.Baumgarte = 0.0f;

    std::vector<ContactManifold> contacts{ manifold };
    SolveContacts(bodies, colliders, contacts, settings, 1.0f / 60.0f);

    REQUIRE(bodies[0].State.LinearVelocity.y >= Catch::Approx(0.0f).margin(1.0e-4f));
    REQUIRE(contacts[0].Points[0].NormalImpulse > 0.0f);
}

TEST_CASE("Contact solver uses exact collider material ids", "[PhysicsEngine][Solver]")
{
    std::vector<RigidBody> bodies;
    bodies.push_back(MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.4f, 0.0f })));
    bodies.push_back(MakeRigidBody(1, StaticBody()));
    bodies[0].State.LinearVelocity = Vec3f{ 0.0f, -2.0f, 0.0f };

    PhysicsMaterial dull;
    dull.Restitution = 0.0f;

    PhysicsMaterial bouncy;
    bouncy.Restitution = 1.0f;

    std::vector<Collider> colliders;
    colliders.push_back(MakeCollider(0, 0, SphereCollider{ 0.25f }, dull, Vec3f{ 10.0f, 0.0f, 0.0f }));
    colliders.push_back(MakeCollider(1, 0, SphereCollider{ 0.5f }, bouncy));
    colliders.push_back(MakeCollider(2, 1, PlaneCollider{ Vec3f::Up(), 0.0f }, bouncy));

    ContactManifold manifold =
        MakeContactManifold(0, 1, 1, 2);

    manifold.Points.push_back(
        MakeContactPoint(
            Vec3f{ 0.0f, 0.0f, 0.0f },
            Vec3f{ 0.0f, -1.0f, 0.0f },
            0.1f));

    PhysicsStepSettings settings;
    settings.VelocityIterations = 1;
    settings.Baumgarte = 0.0f;

    std::vector<ContactManifold> contacts{ manifold };
    SolveContacts(bodies, colliders, contacts, settings, 1.0f / 60.0f);

    REQUIRE(bodies[0].State.LinearVelocity.y == Catch::Approx(2.0f).margin(1.0e-4f));
}

TEST_CASE("Warm starting applies cached normal impulses", "[PhysicsEngine][Solver]")
{
    std::vector<RigidBody> bodies;
    bodies.push_back(MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.4f, 0.0f })));
    bodies.push_back(MakeRigidBody(1, StaticBody()));

    std::vector<Collider> colliders;
    colliders.push_back(MakeCollider(0, 0, SphereCollider{ 0.5f }));
    colliders.push_back(MakeCollider(1, 1, PlaneCollider{ Vec3f::Up(), 0.0f }));

    ContactManifold manifold =
        MakeContactManifold(0, 1, 0, 1);

    manifold.Points.push_back(
        MakeContactPoint(
            Vec3f{ 0.0f, 0.0f, 0.0f },
            Vec3f{ 0.0f, -1.0f, 0.0f },
            0.1f,
            1.0f));

    std::vector<ContactManifold> contacts{ manifold };
    WarmStartContacts(bodies, colliders, contacts);

    REQUIRE(bodies[0].State.LinearVelocity.y > 0.0f);
}

TEST_CASE("Position correction iterates without moving static bodies", "[PhysicsEngine][Solver]")
{
    std::vector<RigidBody> bodies;
    bodies.push_back(MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.4f, 0.0f })));
    bodies.push_back(MakeRigidBody(1, StaticBody(Vec3f::Zero())));

    ContactManifold manifold =
        MakeContactManifold(0, 1, 0, 1);

    manifold.Points.push_back(
        MakeContactPoint(
            Vec3f{ 0.0f, 0.0f, 0.0f },
            Vec3f{ 0.0f, -1.0f, 0.0f },
            0.1f));

    PhysicsStepSettings settings;
    settings.PositionIterations = 2;
    settings.Slop = 0.0f;
    settings.MaxPositionCorrection = 1.0f;

    CorrectPositions(bodies, { manifold }, settings);

    REQUIRE(bodies[0].State.Position.y == Catch::Approx(0.5f).margin(1.0e-4f));
    REQUIRE(bodies[1].State.Position.y == Catch::Approx(0.0f).margin(1.0e-6f));
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
    REQUIRE(!world.BroadphasePairs().empty());
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
    REQUIRE_THROWS_AS(world.RemoveCollider(42), std::out_of_range);
    REQUIRE_THROWS_AS(world.DestroyRigidBody(42), std::out_of_range);

    const BodyID body =
        world.CreateRigidBody(DynamicSphereBody(Vec3f::Zero()));

    REQUIRE_THROWS_AS(world.AddCollider(body, SphereCollider{ -1.0f }), std::invalid_argument);

    RigidBodyDesc invalidDynamic;
    invalidDynamic.Type = BodyType::Dynamic;
    invalidDynamic.State.Position = Vec3f::Zero();
    invalidDynamic.Mass = StaticMassProperties();

    REQUIRE_THROWS_AS(world.CreateRigidBody(invalidDynamic), std::invalid_argument);

    PhysicsWorld invalidSettingsWorld;
    invalidSettingsWorld.Settings.PositionIterations = 0;
    REQUIRE_THROWS_AS(invalidSettingsWorld.Step(1.0f / 60.0f), std::invalid_argument);
}
