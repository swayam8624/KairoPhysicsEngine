#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

import Kairo.Foundation.PhysicsEngine;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Math.Quaternion;
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

    RigidBodyDesc DynamicBoxBody(
        const Vec3f& position,
        const Vec3f& halfExtents = Vec3f{ 0.5f, 0.5f, 0.5f })
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Dynamic;
        desc.State.Position = position;
        desc.Mass = BoxMassProperties(halfExtents, 1.0f);
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

TEST_CASE("Broadphase respects belongs-to and collides-with filters", "[PhysicsEngine][Broadphase]")
{
    std::vector<RigidBody> bodies;
    bodies.push_back(MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.5f, 0.0f })));
    bodies.push_back(MakeRigidBody(1, DynamicSphereBody(Vec3f{ 0.6f, 0.5f, 0.0f })));

    std::vector<Collider> colliders;
    colliders.push_back(MakeCollider(0, 0, SphereCollider{ 0.5f }));
    colliders.push_back(MakeCollider(1, 1, SphereCollider{ 0.5f }));

    colliders[0].BelongsTo = 0b0001u;
    colliders[0].CollidesWith = 0b0100u;
    colliders[1].BelongsTo = 0b0010u;
    colliders[1].CollidesWith = 0b0001u;

    REQUIRE(ComputeBroadphasePairs(bodies, colliders).empty());

    colliders[0].CollidesWith = 0b0010u;
    REQUIRE(ComputeBroadphasePairs(bodies, colliders).size() == 1);
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

TEST_CASE("Narrowphase keeps plane contact normals stable when pairs are swapped", "[PhysicsEngine][Narrowphase]")
{
    const RigidBody sphereBody =
        MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.4f, 0.0f }));

    const RigidBody planeBody =
        MakeRigidBody(1, StaticBody());

    const Collider sphere =
        MakeCollider(0, 0, SphereCollider{ 0.5f });

    const Collider plane =
        MakeCollider(1, 1, PlaneCollider{ Vec3f::Up(), 0.0f });

    const auto spherePlane =
        CollidePair(sphereBody, sphere, planeBody, plane);

    const auto planeSphere =
        CollidePair(planeBody, plane, sphereBody, sphere);

    REQUIRE(spherePlane.has_value());
    REQUIRE(planeSphere.has_value());
    REQUIRE(spherePlane->Points[0].Normal.y == Catch::Approx(-1.0f));
    REQUIRE(planeSphere->Points[0].Normal.y == Catch::Approx(1.0f));

    const RigidBody boxBody =
        MakeRigidBody(2, StaticBody(Vec3f{ 0.0f, 0.4f, 0.0f }));

    const Collider box =
        MakeCollider(2, 2, AABBCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });

    const auto boxPlane =
        CollidePair(boxBody, box, planeBody, plane);

    const auto planeBox =
        CollidePair(planeBody, plane, boxBody, box);

    REQUIRE(boxPlane.has_value());
    REQUIRE(planeBox.has_value());
    REQUIRE(boxPlane->Points[0].Normal.y == Catch::Approx(-1.0f));
    REQUIRE(planeBox->Points[0].Normal.y == Catch::Approx(1.0f));
}

TEST_CASE("Sphere sphere contact point is midpoint of surface points", "[PhysicsEngine][Narrowphase]")
{
    const RigidBody a =
        MakeRigidBody(0, DynamicSphereBody(Vec3f::Zero()));

    const RigidBody b =
        MakeRigidBody(1, DynamicSphereBody(Vec3f{ 0.75f, 0.0f, 0.0f }));

    const Collider ca =
        MakeCollider(0, 0, SphereCollider{ 0.5f });

    const Collider cb =
        MakeCollider(1, 1, SphereCollider{ 0.5f });

    const auto contact =
        CollidePair(a, ca, b, cb);

    REQUIRE(contact.has_value());
    REQUIRE(contact->Points[0].Position.x == Catch::Approx(0.375f));
    REQUIRE(contact->Points[0].Normal.x == Catch::Approx(1.0f));
}

TEST_CASE("Trigger contacts are reported but not solved", "[PhysicsEngine][Narrowphase][Solver]")
{
    std::vector<RigidBody> bodies;
    bodies.push_back(MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.4f, 0.0f })));
    bodies.push_back(MakeRigidBody(1, StaticBody()));
    bodies[0].State.LinearVelocity = Vec3f{ 0.0f, -2.0f, 0.0f };

    std::vector<Collider> colliders;
    colliders.push_back(MakeCollider(0, 0, SphereCollider{ 0.5f }));
    colliders.push_back(MakeCollider(1, 1, PlaneCollider{ Vec3f::Up(), 0.0f }));
    colliders[1].IsTrigger = true;

    const auto contact =
        CollidePair(bodies[0], colliders[0], bodies[1], colliders[1]);

    REQUIRE(contact.has_value());
    REQUIRE(contact->IsTrigger);

    std::vector<ContactManifold> contacts{ *contact };
    PhysicsStepSettings settings;
    settings.Baumgarte = 0.0f;

    SolveContacts(bodies, colliders, contacts, settings, 1.0f / 60.0f);
    CorrectPositions(bodies, contacts, settings);

    REQUIRE(bodies[0].State.LinearVelocity.y == Catch::Approx(-2.0f));
    REQUIRE(bodies[0].State.Position.y == Catch::Approx(0.4f));
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

TEST_CASE("Contact friction reduces tangential velocity", "[PhysicsEngine][Solver]")
{
    std::vector<RigidBody> bodies;
    bodies.push_back(MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.4f, 0.0f })));
    bodies.push_back(MakeRigidBody(1, StaticBody()));
    bodies[0].State.LinearVelocity = Vec3f{ 2.0f, -1.0f, 0.0f };

    PhysicsMaterial material;
    material.Restitution = 0.0f;
    material.DynamicFriction = 1.0f;

    std::vector<Collider> colliders;
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
    settings.VelocityIterations = 8;
    settings.Baumgarte = 0.0f;

    std::vector<ContactManifold> contacts{ manifold };
    SolveContacts(bodies, colliders, contacts, settings, 1.0f / 60.0f);

    REQUIRE(std::abs(bodies[0].State.LinearVelocity.x) < 2.0f);
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

TEST_CASE("PhysicsWorld resolves dynamic object collisions", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;
    world.Settings.VelocityIterations = 12;
    world.Settings.PositionIterations = 6;

    RigidBodyDesc leftDesc =
        DynamicSphereBody(Vec3f{ -0.65f, 1.0f, 0.0f }, 0.5f);

    RigidBodyDesc rightDesc =
        DynamicSphereBody(Vec3f{ 0.65f, 1.0f, 0.0f }, 0.5f);

    leftDesc.EnableGravity = false;
    rightDesc.EnableGravity = false;
    leftDesc.State.LinearVelocity = Vec3f{ 2.0f, 0.0f, 0.0f };
    rightDesc.State.LinearVelocity = Vec3f{ -2.0f, 0.0f, 0.0f };

    const BodyID left =
        world.CreateRigidBody(leftDesc);

    const BodyID right =
        world.CreateRigidBody(rightDesc);

    PhysicsMaterial bouncy;
    bouncy.Restitution = 1.0f;
    bouncy.StaticFriction = 0.0f;
    bouncy.DynamicFriction = 0.0f;

    [[maybe_unused]] const ColliderID leftCollider =
        world.AddCollider(left, SphereCollider{ 0.5f }, bouncy);

    [[maybe_unused]] const ColliderID rightCollider =
        world.AddCollider(right, SphereCollider{ 0.5f }, bouncy);

    bool contacted =
        false;

    for (int i = 0; i < 24; ++i)
    {
        world.Step(1.0f / 60.0f);
        contacted =
            contacted || !world.Contacts().empty();
    }

    REQUIRE(contacted);
    REQUIRE(world.Bodies()[left].State.Position.x < world.Bodies()[right].State.Position.x);
    REQUIRE(world.Bodies()[right].State.Position.x - world.Bodies()[left].State.Position.x >= 0.95f);
}

TEST_CASE("PhysicsWorld resolves dynamic sphere box collisions", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;
    world.Settings.VelocityIterations = 12;
    world.Settings.PositionIterations = 8;

    RigidBodyDesc sphereDesc =
        DynamicSphereBody(Vec3f{ -0.35f, 1.0f, 0.0f }, 0.5f);

    RigidBodyDesc boxDesc =
        DynamicBoxBody(Vec3f{ 0.35f, 1.0f, 0.0f }, Vec3f{ 0.45f, 0.45f, 0.5f });

    sphereDesc.EnableGravity = false;
    boxDesc.EnableGravity = false;

    const BodyID sphere =
        world.CreateRigidBody(sphereDesc);

    const BodyID box =
        world.CreateRigidBody(boxDesc);

    [[maybe_unused]] const ColliderID sphereCollider =
        world.AddCollider(sphere, SphereCollider{ 0.5f });

    [[maybe_unused]] const ColliderID boxCollider =
        world.AddCollider(box, BoxCollider{ Vec3f{ 0.45f, 0.45f, 0.5f } });

    world.Step(1.0f / 60.0f);

    REQUIRE(!world.Contacts().empty());
    REQUIRE(world.Bodies()[sphere].State.Position.x < -0.35f);
    REQUIRE(world.Bodies()[box].State.Position.x > 0.35f);
}

TEST_CASE("Kinematic bodies advance by authored velocity only", "[PhysicsEngine][World]")
{
    PhysicsWorld world;

    RigidBodyDesc desc;
    desc.Type = BodyType::Kinematic;
    desc.State.Position = Vec3f::Zero();
    desc.State.LinearVelocity = Vec3f{ 2.0f, 0.0f, 0.0f };

    const BodyID body =
        world.CreateRigidBody(desc);

    world.Step(0.5f);

    REQUIRE(world.Bodies()[body].State.Position.x == Catch::Approx(1.0f));
    REQUIRE(world.Bodies()[body].State.Position.y == Catch::Approx(0.0f));
}

TEST_CASE("Dynamic body damping and velocity clamps are applied", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();

    RigidBodyDesc desc =
        DynamicSphereBody(Vec3f::Zero());

    desc.EnableGravity = false;
    desc.LinearDamping = 0.0f;
    desc.MaxLinearSpeed = 10.0f;
    desc.State.LinearVelocity = Vec3f{ 100.0f, 0.0f, 0.0f };

    const BodyID body =
        world.CreateRigidBody(desc);

    world.Step(1.0f / 60.0f);

    REQUIRE(world.Bodies()[body].State.LinearVelocity.Length() == Catch::Approx(10.0f).margin(1.0e-4f));
    REQUIRE(world.LastStepProfile().StepMs >= 0.0);
    REQUIRE(world.LastStepProfile().BroadphaseMs >= 0.0);
    REQUIRE(world.LastStepProfile().NarrowphaseMs >= 0.0);
    REQUIRE(world.LastStepProfile().SolverMs >= 0.0);
}

TEST_CASE("Sleeping bodies stop integrating and wake on force", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.SleepTime = 1.0f / 120.0f;
    world.Settings.SleepLinearSpeed = 0.1f;
    world.Settings.SleepAngularSpeed = 0.1f;

    RigidBodyDesc desc =
        DynamicSphereBody(Vec3f::Zero());

    desc.EnableGravity = false;
    desc.AllowSleeping = true;

    const BodyID body =
        world.CreateRigidBody(desc);

    world.Step(1.0f / 60.0f);

    REQUIRE(world.Bodies()[body].Sleeping);

    AddForce(world.Bodies()[body].Forces, Vec3f{ 10.0f, 0.0f, 0.0f });
    world.Step(1.0f / 60.0f);

    REQUIRE(!world.Bodies()[body].Sleeping);
    REQUIRE(world.Bodies()[body].State.LinearVelocity.x > 0.0f);
}

TEST_CASE("World force and impulse APIs mutate dynamic bodies", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    RigidBodyDesc desc =
        DynamicSphereBody(Vec3f::Zero());

    desc.EnableGravity = false;

    const BodyID body =
        world.CreateRigidBody(desc);

    world.AddBodyForceAtPoint(
        body,
        Vec3f{ 0.0f, 10.0f, 0.0f },
        Vec3f{ 1.0f, 0.0f, 0.0f });

    world.Step(1.0f / 60.0f);

    REQUIRE(world.Bodies()[body].State.AngularVelocity.z > 0.0f);

    world.ApplyBodyImpulseAtPoint(
        body,
        Vec3f{ 1.0f, 0.0f, 0.0f },
        Vec3f::Zero());

    REQUIRE(world.Bodies()[body].State.LinearVelocity.x > 0.0f);
    REQUIRE_THROWS_AS(world.AddBodyForce(99, Vec3f::UnitX()), std::out_of_range);
}

TEST_CASE("World overlap queries return active finite colliders", "[PhysicsEngine][World]")
{
    PhysicsWorld world;

    const BodyID sphere =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 0.0f, 0.5f, 0.0f }));

    const BodyID floor =
        world.CreateRigidBody(StaticBody());

    const ColliderID sphereCollider =
        world.AddCollider(sphere, SphereCollider{ 0.5f });

    [[maybe_unused]] const ColliderID planeCollider =
        world.AddCollider(floor, PlaneCollider{ Vec3f::Up(), 0.0f });

    const std::vector<ColliderID> aabbHits =
        world.QueryAABB(AABBf::FromCenterExtent(Vec3f{ 0.0f, 0.5f, 0.0f }, Vec3f{ 1.0f, 1.0f, 1.0f }));

    const std::vector<ColliderID> sphereHits =
        world.QuerySphere(Vec3f{ 0.0f, 0.5f, 0.0f }, 1.0f);

    REQUIRE(aabbHits.size() == 1);
    REQUIRE(aabbHits[0] == sphereCollider);
    REQUIRE(sphereHits.size() == 1);
    REQUIRE(sphereHits[0] == sphereCollider);
}

TEST_CASE("Collision layer responses choose ignore trigger or block", "[PhysicsEngine][World][CollisionRules]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    const BodyID player =
        world.CreateRigidBody(StaticBody(Vec3f::Zero()));

    const BodyID pickup =
        world.CreateRigidBody(StaticBody(Vec3f{ 0.75f, 0.0f, 0.0f }));

    const ColliderID playerCollider =
        world.AddCollider(player, SphereCollider{ 0.5f });

    const ColliderID pickupCollider =
        world.AddCollider(pickup, SphereCollider{ 0.5f });

    world.SetColliderCollisionLayer(playerCollider, CollisionLayer::Player);
    world.SetColliderCollisionLayer(pickupCollider, CollisionLayer::Trigger);
    world.SetCollisionLayerResponse(CollisionLayer::Player, CollisionLayer::Trigger, CollisionResponse::Ignore);

    world.Step(1.0f / 60.0f);
    REQUIRE(world.Contacts().empty());
    REQUIRE(world.ContactEvents().empty());

    world.SetCollisionLayerResponse(CollisionLayer::Player, CollisionLayer::Trigger, CollisionResponse::Trigger);
    world.Step(1.0f / 60.0f);

    REQUIRE(world.Contacts().size() == 1);
    REQUIRE(world.Contacts()[0].IsTrigger);
    REQUIRE(world.ContactEvents().size() == 1);
    REQUIRE(world.ContactEvents()[0].IsTrigger);
    REQUIRE(world.ContactEvents()[0].Response == CollisionResponse::Trigger);

    world.SetCollisionLayerResponse(CollisionLayer::Player, CollisionLayer::Trigger, CollisionResponse::Block);
    world.Step(1.0f / 60.0f);

    REQUIRE(world.Contacts().size() == 1);
    REQUIRE_FALSE(world.Contacts()[0].IsTrigger);
    REQUIRE(world.ContactEvents()[0].Response == CollisionResponse::Block);
}

TEST_CASE("Collision pair response overrides layer response", "[PhysicsEngine][World][CollisionRules]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();

    const BodyID projectile =
        world.CreateRigidBody(StaticBody(Vec3f::Zero()));

    const BodyID owner =
        world.CreateRigidBody(StaticBody(Vec3f{ 0.75f, 0.0f, 0.0f }));

    const ColliderID projectileCollider =
        world.AddCollider(projectile, SphereCollider{ 0.5f });

    const ColliderID ownerCollider =
        world.AddCollider(owner, SphereCollider{ 0.5f });

    world.SetColliderCollisionLayer(projectileCollider, CollisionLayer::Projectile);
    world.SetColliderCollisionLayer(ownerCollider, CollisionLayer::Player);
    world.SetCollisionLayerResponse(CollisionLayer::Projectile, CollisionLayer::Player, CollisionResponse::Ignore);
    world.SetCollisionPairResponse(projectileCollider, ownerCollider, CollisionResponse::Block);

    world.Step(1.0f / 60.0f);

    REQUIRE(world.Contacts().size() == 1);
    REQUIRE_FALSE(world.Contacts()[0].IsTrigger);
    REQUIRE(world.ContactEvents()[0].Response == CollisionResponse::Block);

    world.ClearCollisionPairResponse(projectileCollider, ownerCollider);
    world.Step(1.0f / 60.0f);

    REQUIRE(world.Contacts().empty());
}

TEST_CASE("Collision filter callback can classify pairs at runtime", "[PhysicsEngine][World][CollisionRules]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();

    const BodyID sensor =
        world.CreateRigidBody(StaticBody(Vec3f::Zero()));

    const BodyID target =
        world.CreateRigidBody(StaticBody(Vec3f{ 0.75f, 0.0f, 0.0f }));

    const ColliderID sensorCollider =
        world.AddCollider(sensor, SphereCollider{ 0.5f });

    const ColliderID targetCollider =
        world.AddCollider(target, SphereCollider{ 0.5f });

    world.SetCollisionFilterCallback(
        [sensorCollider, targetCollider](const Collider& a, const Collider& b)
        {
            const bool selectedPair =
                (a.ID == sensorCollider && b.ID == targetCollider) ||
                (a.ID == targetCollider && b.ID == sensorCollider);

            return selectedPair
                ? CollisionResponse::Trigger
                : CollisionResponse::Ignore;
        });

    world.Step(1.0f / 60.0f);

    REQUIRE(world.Contacts().size() == 1);
    REQUIRE(world.Contacts()[0].IsTrigger);
    REQUIRE(world.ContactEvents()[0].Response == CollisionResponse::Trigger);
}

TEST_CASE("World raycasts return nearest and sorted collider hits", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();

    const BodyID nearBody =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 0.0f, 1.0f, 0.0f }, 0.5f));

    const BodyID farBody =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 2.0f, 1.0f, 0.0f }, 0.5f));

    const BodyID boxBody =
        world.CreateRigidBody(DynamicBoxBody(Vec3f{ 4.0f, 1.0f, 0.0f }, Vec3f{ 0.5f, 0.5f, 0.5f }));

    const ColliderID nearCollider =
        world.AddCollider(nearBody, SphereCollider{ 0.5f });

    const ColliderID farCollider =
        world.AddCollider(farBody, SphereCollider{ 0.5f });

    const ColliderID boxCollider =
        world.AddCollider(boxBody, BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });

    const auto nearest =
        world.Raycast(Vec3f{ -2.0f, 1.0f, 0.0f }, Vec3f::UnitX(), 8.0f);

    REQUIRE(nearest.has_value());
    REQUIRE(nearest->Collider == nearCollider);
    REQUIRE(nearest->Distance == Catch::Approx(1.5f));
    REQUIRE(nearest->Point.x == Catch::Approx(-0.5f));
    REQUIRE(nearest->Normal.x == Catch::Approx(-1.0f));

    const std::vector<PhysicsRayHit> allHits =
        world.RaycastAll(Vec3f{ -2.0f, 1.0f, 0.0f }, Vec3f::UnitX(), 8.0f);

    REQUIRE(allHits.size() == 3);
    REQUIRE(allHits[0].Collider == nearCollider);
    REQUIRE(allHits[1].Collider == farCollider);
    REQUIRE(allHits[2].Collider == boxCollider);
    REQUIRE(allHits[0].Distance < allHits[1].Distance);
    REQUIRE(allHits[1].Distance < allHits[2].Distance);

    const auto ignored =
        world.Raycast(Vec3f{ -2.0f, 1.0f, 0.0f }, Vec3f::UnitX(), 8.0f, 0xFFFF'FFFFu, nearCollider);

    REQUIRE(ignored.has_value());
    REQUIRE(ignored->Collider == farCollider);
}

TEST_CASE("World raycasts validate inputs and respect max distance", "[PhysicsEngine][World][Validation]")
{
    PhysicsWorld world;

    const BodyID body =
        world.CreateRigidBody(DynamicSphereBody(Vec3f::Zero()));

    [[maybe_unused]] const ColliderID collider =
        world.AddCollider(body, SphereCollider{ 0.5f });

    REQUIRE_THROWS_AS(world.Raycast(Vec3f::Zero(), Vec3f::Zero()), std::invalid_argument);
    REQUIRE_THROWS_AS(world.Raycast(Vec3f::Zero(), Vec3f::UnitX(), 0.0f), std::invalid_argument);
    REQUIRE_FALSE(world.Raycast(Vec3f{ -2.0f, 0.0f, 0.0f }, Vec3f::UnitX(), 1.0f).has_value());
}

TEST_CASE("World accelerated overlap queries and rays track direct transform edits", "[PhysicsEngine][World][Queries]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();

    const BodyID nearBody =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 0.0f, 1.0f, 0.0f }, 0.5f));
    const BodyID farBody =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 20.0f, 1.0f, 0.0f }, 0.5f));
    const BodyID planeBody =
        world.CreateRigidBody(StaticBody());

    const ColliderID nearCollider =
        world.AddCollider(nearBody, SphereCollider{ 0.5f });
    [[maybe_unused]] const ColliderID farCollider =
        world.AddCollider(farBody, SphereCollider{ 0.5f });
    const ColliderID planeCollider =
        world.AddCollider(planeBody, PlaneCollider{ Vec3f::Up(), 0.0f });

    REQUIRE(world.QueryAABB(
        AABBf::FromCenterExtent(Vec3f{ 0.0f, 1.0f, 0.0f }, Vec3f{ 1.0f, 1.0f, 1.0f })) ==
        std::vector<ColliderID>{ nearCollider });
    REQUIRE(world.QuerySphere(Vec3f{ 0.0f, 1.0f, 0.0f }, 1.0f) ==
        std::vector<ColliderID>{ nearCollider });

    const std::vector<PhysicsRayHit> initialHits =
        world.RaycastAll(Vec3f{ -2.0f, 1.0f, 0.0f }, Vec3f::UnitX(), 4.0f);
    REQUIRE(initialHits.size() == 1u);
    REQUIRE(initialHits.front().Collider == nearCollider);

    world.Bodies()[nearBody].State.Position = Vec3f{ 5.0f, 1.0f, 0.0f };
    REQUIRE_FALSE(world.Raycast(Vec3f{ -2.0f, 1.0f, 0.0f }, Vec3f::UnitX(), 4.0f).has_value());

    const auto planeHit =
        world.Raycast(Vec3f{ 0.0f, 2.0f, 0.0f }, -Vec3f::Up(), 4.0f);
    REQUIRE(planeHit.has_value());
    REQUIRE(planeHit->Collider == planeCollider);
}

TEST_CASE("World reports deterministic contact begin stay and end events", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;
    std::vector<PhysicsContactEvent> callbackEvents;
    world.SetContactEventCallback(
        [&callbackEvents](const PhysicsContactEvent& event)
        {
            callbackEvents.push_back(event);
        });

    const BodyID a =
        world.CreateRigidBody(StaticBody(Vec3f::Zero()));

    const BodyID b =
        world.CreateRigidBody(StaticBody(Vec3f{ 0.75f, 0.0f, 0.0f }));

    [[maybe_unused]] const ColliderID ca =
        world.AddCollider(a, SphereCollider{ 0.5f });

    [[maybe_unused]] const ColliderID cb =
        world.AddCollider(b, SphereCollider{ 0.5f });

    world.Step(1.0f / 60.0f);
    REQUIRE(world.ContactEvents().size() == 1);
    REQUIRE(world.ContactEvents()[0].Type == PhysicsContactEventType::Begin);
    REQUIRE(callbackEvents.back().Type == PhysicsContactEventType::Begin);

    world.Step(1.0f / 60.0f);
    REQUIRE(world.ContactEvents().size() == 1);
    REQUIRE(world.ContactEvents()[0].Type == PhysicsContactEventType::Stay);
    REQUIRE(callbackEvents.back().Type == PhysicsContactEventType::Stay);

    world.Bodies()[b].State.Position = Vec3f{ 4.0f, 0.0f, 0.0f };
    world.Step(1.0f / 60.0f);

    REQUIRE(world.ContactEvents().size() == 1);
    REQUIRE(world.ContactEvents()[0].Type == PhysicsContactEventType::End);
    REQUIRE(callbackEvents.back().Type == PhysicsContactEventType::End);
    REQUIRE(callbackEvents.size() == 3);
}

TEST_CASE("Fixed stepping is deterministic for replay-equivalent worlds", "[PhysicsEngine][World]")
{
    auto makeWorld = []
    {
        PhysicsWorld world;
        world.Settings.EnableSleeping = false;

        const BodyID sphere =
            world.CreateRigidBody(DynamicSphereBody(Vec3f{ 0.0f, 2.0f, 0.0f }));

        const BodyID floor =
            world.CreateRigidBody(StaticBody());

        [[maybe_unused]] const ColliderID sphereCollider =
            world.AddCollider(sphere, SphereCollider{ 0.5f });

        [[maybe_unused]] const ColliderID floorCollider =
            world.AddCollider(floor, PlaneCollider{ Vec3f::Up(), 0.0f });

        return world;
    };

    PhysicsWorld a =
        makeWorld();

    PhysicsWorld b =
        makeWorld();

    for (int i = 0; i < 120; ++i)
    {
        a.Step(1.0f / 60.0f);
        b.Step(1.0f / 60.0f);
    }

    REQUIRE(a.Bodies()[0].State.Position.x == Catch::Approx(b.Bodies()[0].State.Position.x));
    REQUIRE(a.Bodies()[0].State.Position.y == Catch::Approx(b.Bodies()[0].State.Position.y));
    REQUIRE(a.Bodies()[0].State.LinearVelocity.y == Catch::Approx(b.Bodies()[0].State.LinearVelocity.y));
}

TEST_CASE("Fixed timestep accumulator advances deterministic substeps", "[PhysicsEngine][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    RigidBodyDesc desc =
        DynamicSphereBody(Vec3f::Zero());

    desc.EnableGravity = false;
    desc.State.LinearVelocity = Vec3f{ 1.0f, 0.0f, 0.0f };

    const BodyID body =
        world.CreateRigidBody(desc);

    constexpr float fixedDt = 1.0f / 60.0f;

    REQUIRE(world.StepFixed(fixedDt * 0.5f, fixedDt) == 0);
    REQUIRE(world.Bodies()[body].State.Position.x == Catch::Approx(0.0f));
    REQUIRE(world.FixedAccumulator() == Catch::Approx(fixedDt * 0.5f));

    REQUIRE(world.StepFixed(fixedDt * 0.5f, fixedDt) == 1);
    REQUIRE(world.Bodies()[body].State.Position.x == Catch::Approx(fixedDt).margin(1.0e-6f));

    world.ResetFixedAccumulator();
    REQUIRE(world.FixedAccumulator() == Catch::Approx(0.0f));
    REQUIRE_THROWS_AS(world.StepFixed(fixedDt, fixedDt, 0), std::invalid_argument);
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

TEST_CASE("Rotated BoxCollider uses SAT contacts", "[PhysicsEngine][Narrowphase]")
{
    const RigidBody a =
        MakeRigidBody(0, StaticBody(Vec3f::Zero()));

    const RigidBody b =
        MakeRigidBody(1, StaticBody(Vec3f{ 0.75f, 0.0f, 0.0f }));

    const Collider ca =
        MakeCollider(
            0,
            0,
            BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });

    const Collider cb =
        MakeCollider(
            1,
            1,
            BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } },
            {},
            Vec3f::Zero(),
            RotationAroundZ(0.35f));

    const auto contact =
        CollidePair(a, ca, b, cb);

    REQUIRE(contact.has_value());
    REQUIRE(contact->Points.size() == 1);
    REQUIRE(contact->Points[0].PenetrationDepth > 0.0f);
}

TEST_CASE("Rotated BoxCollider contacts planes through projected radius", "[PhysicsEngine][Narrowphase]")
{
    const RigidBody boxBody =
        MakeRigidBody(0, StaticBody(Vec3f{ 0.0f, 0.45f, 0.0f }));

    const RigidBody planeBody =
        MakeRigidBody(1, StaticBody());

    const Collider box =
        MakeCollider(
            0,
            0,
            BoxCollider{ Vec3f{ 0.5f, 0.25f, 0.5f } },
            {},
            Vec3f::Zero(),
            RotationAroundZ(0.5f));

    const Collider plane =
        MakeCollider(1, 1, PlaneCollider{ Vec3f::Up(), 0.0f });

    const auto contact =
        CollidePair(boxBody, box, planeBody, plane);

    REQUIRE(contact.has_value());
    REQUIRE(contact->Points[0].Normal.y < 0.0f);
}

TEST_CASE("Sphere and BoxCollider contacts are detected in both pair orders", "[PhysicsEngine][Narrowphase]")
{
    const RigidBody sphereBody =
        MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.85f, 0.0f }));

    const RigidBody boxBody =
        MakeRigidBody(1, StaticBody(Vec3f::Zero()));

    const Collider sphere =
        MakeCollider(0, 0, SphereCollider{ 0.5f });

    const Collider box =
        MakeCollider(
            1,
            1,
            BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } },
            {},
            Vec3f::Zero(),
            RotationAroundZ(0.2f));

    const auto sphereBox =
        CollidePair(sphereBody, sphere, boxBody, box);

    const auto boxSphere =
        CollidePair(boxBody, box, sphereBody, sphere);

    REQUIRE(sphereBox.has_value());
    REQUIRE(boxSphere.has_value());
    REQUIRE(sphereBox->Points[0].PenetrationDepth > 0.0f);
    REQUIRE(boxSphere->Points[0].PenetrationDepth > 0.0f);
    REQUIRE(sphereBox->Points[0].Normal.y < 0.0f);
    REQUIRE(boxSphere->Points[0].Normal.y > 0.0f);
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

    const ColliderID validCollider =
        world.AddCollider(body, SphereCollider{ 0.5f });

    REQUIRE_THROWS_AS(world.AddCollider(body, SphereCollider{ -1.0f }), std::invalid_argument);
    REQUIRE_THROWS_AS(world.AddCollider(body, BoxCollider{ Vec3f{ -1.0f, 1.0f, 1.0f } }), std::invalid_argument);
    REQUIRE_THROWS_AS(world.SetCollisionLayerResponse(0u, CollisionLayer::Default, CollisionResponse::Block), std::invalid_argument);
    REQUIRE_THROWS_AS(world.SetCollisionPairResponse(validCollider, validCollider, CollisionResponse::Ignore), std::invalid_argument);
    REQUIRE_THROWS_AS(world.SetCollisionPairResponse(validCollider, 99, CollisionResponse::Ignore), std::out_of_range);

    RigidBodyDesc invalidDynamic;
    invalidDynamic.Type = BodyType::Dynamic;
    invalidDynamic.State.Position = Vec3f::Zero();
    invalidDynamic.Mass = StaticMassProperties();

    REQUIRE_THROWS_AS(world.CreateRigidBody(invalidDynamic), std::invalid_argument);

    PhysicsWorld invalidSettingsWorld;
    invalidSettingsWorld.Settings.PositionIterations = 0;
    REQUIRE_THROWS_AS(invalidSettingsWorld.Step(1.0f / 60.0f), std::invalid_argument);
}
