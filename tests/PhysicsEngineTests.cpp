#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <variant>
#include <filesystem>
#include <fstream>

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

    TriangleMeshCollider FloorMesh(float halfExtent = 2.0f)
    {
        TriangleMeshCollider mesh;
        mesh.Vertices = {
  { -halfExtent, 0.0f, -halfExtent },
  {  halfExtent, 0.0f, -halfExtent },
  {  halfExtent, 0.0f,  halfExtent },
  { -halfExtent, 0.0f,  halfExtent }
        };
        // Counter-clockwise as viewed from +Y.
        mesh.Triangles = { { 0u, 2u, 1u }, { 0u, 3u, 2u } };
        return mesh;
    }

    ConvexHullCollider CubeHull(float halfExtent = 0.5f)
    {
        return {
            {
                { -halfExtent, -halfExtent, -halfExtent },
                {  halfExtent, -halfExtent, -halfExtent },
                {  halfExtent,  halfExtent, -halfExtent },
                { -halfExtent,  halfExtent, -halfExtent },
                { -halfExtent, -halfExtent,  halfExtent },
                {  halfExtent, -halfExtent,  halfExtent },
                {  halfExtent,  halfExtent,  halfExtent },
                { -halfExtent,  halfExtent,  halfExtent }
            },
            {
                { 0u, 1u, 2u }, { 0u, 2u, 3u },
                { 4u, 6u, 5u }, { 4u, 7u, 6u },
                { 0u, 3u, 7u }, { 0u, 7u, 4u },
                { 1u, 5u, 6u }, { 1u, 6u, 2u },
                { 0u, 4u, 5u }, { 0u, 5u, 1u },
                { 3u, 2u, 6u }, { 3u, 6u, 7u }
            }
        };
    }
}




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

TEST_CASE("Continuous sphere rigid body does not tunnel through a plane",
    "[PhysicsEngine][CCD]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    const BodyID floorBody = world.CreateRigidBody(StaticBody());
    [[maybe_unused]] const ColliderID floorCollider =
        world.AddCollider(floorBody, PlaneCollider{ Vec3f::Up(), 0.0f });

    RigidBodyDesc fastDesc =
        DynamicSphereBody(Vec3f{ 0.0f, 2.0f, 0.0f }, 0.25f);
    fastDesc.State.LinearVelocity = Vec3f{ 0.0f, -200.0f, 0.0f };
    fastDesc.CollisionDetection = CollisionDetectionMode::Continuous;
    const BodyID fastBody = world.CreateRigidBody(fastDesc);
    [[maybe_unused]] const ColliderID fastCollider =
        world.AddCollider(fastBody, SphereCollider{ 0.25f });

    world.Step(1.0f / 60.0f);

    CHECK(world.Bodies().at(fastBody).State.Position.y >= 0.24f);
    CHECK(world.Bodies().at(fastBody).State.LinearVelocity.y >= 0.0f);
}

TEST_CASE("Continuous boxes use conservative shape bounds against thin walls",
    "[PhysicsEngine][CCD]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    const BodyID wallBody = world.CreateRigidBody(StaticBody());
    [[maybe_unused]] const ColliderID wallCollider =
        world.AddCollider(
            wallBody,
            AABBCollider{ Vec3f{ 0.05f, 2.0f, 2.0f } });

    const Vec3f halfExtents{ 0.2f, 0.2f, 0.2f };
    RigidBodyDesc fastDesc =
        DynamicBoxBody(Vec3f{ -2.0f, 0.0f, 0.0f }, halfExtents);
    fastDesc.State.LinearVelocity = Vec3f{ 200.0f, 0.0f, 0.0f };
    fastDesc.CollisionDetection = CollisionDetectionMode::Continuous;
    const BodyID fastBody = world.CreateRigidBody(fastDesc);
    [[maybe_unused]] const ColliderID fastCollider =
        world.AddCollider(fastBody, BoxCollider{ halfExtents });

    world.Step(0.02f);

    CHECK(world.Bodies().at(fastBody).State.Position.x < -0.05f);
    CHECK(world.Bodies().at(fastBody).State.LinearVelocity.x <= 0.0f);
}

TEST_CASE("Continuous rigid body CCD uses relative motion for dynamic targets",
    "[PhysicsEngine][CCD]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    RigidBodyDesc aDesc =
        DynamicSphereBody(Vec3f{ -1.0f, 0.0f, 0.0f }, 0.2f);
    aDesc.State.LinearVelocity = Vec3f{ 100.0f, 0.0f, 0.0f };
    aDesc.CollisionDetection = CollisionDetectionMode::Continuous;
    const BodyID a = world.CreateRigidBody(aDesc);
    [[maybe_unused]] const ColliderID colliderA =
        world.AddCollider(a, SphereCollider{ 0.2f });

    RigidBodyDesc bDesc =
        DynamicSphereBody(Vec3f{ 1.0f, 0.0f, 0.0f }, 0.2f);
    bDesc.State.LinearVelocity = Vec3f{ -100.0f, 0.0f, 0.0f };
    const BodyID b = world.CreateRigidBody(bDesc);
    [[maybe_unused]] const ColliderID colliderB =
        world.AddCollider(b, SphereCollider{ 0.2f });

    world.Step(0.012f);

    const float separation =
        world.Bodies().at(b).State.Position.x -
        world.Bodies().at(a).State.Position.x;
    CHECK(separation >= 0.39f);
    CHECK(world.Bodies().at(a).State.Position.x <
        world.Bodies().at(b).State.Position.x);
    CHECK(world.Bodies().at(a).State.LinearVelocity.x <= 0.0f);
    CHECK(world.Bodies().at(b).State.LinearVelocity.x >= 0.0f);
}

TEST_CASE("Box plane contact produces a stable four-point face manifold",
    "[PhysicsEngine][Manifold][Box][Plane]")
{
    const RigidBody boxBody =
        MakeRigidBody(0, DynamicBoxBody(Vec3f{ 0.0f, 0.45f, 0.0f }));
    const RigidBody planeBody = MakeRigidBody(1, StaticBody());
    const Collider box =
        MakeCollider(0, 0, BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });
    const Collider plane =
        MakeCollider(1, 1, PlaneCollider{ Vec3f::Up(), 0.0f });

    const auto contact = CollidePair(boxBody, box, planeBody, plane);
    REQUIRE(contact.has_value());
    REQUIRE(contact->Points.size() == 4u);
    for (const ContactPoint& point : contact->Points)
    {
        CHECK(point.Normal.y < -0.99f);
        CHECK(point.PenetrationDepth == Catch::Approx(0.05f).margin(1.0e-4f));
    }
}

TEST_CASE("Oriented box and AABB use the shared SAT manifold path",
    "[PhysicsEngine][Manifold][Box][AABB]")
{
    RigidBodyDesc orientedDesc = DynamicBoxBody(Vec3f::Zero());
    orientedDesc.State.Rotation = RotationAroundZ(0.15f);
    const RigidBody orientedBody = MakeRigidBody(0, orientedDesc);
    const RigidBody axisBody =
        MakeRigidBody(1, StaticBody(Vec3f{ 0.80f, 0.0f, 0.0f }));
    const Collider oriented =
        MakeCollider(0, 0, BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });
    const Collider axis =
        MakeCollider(1, 1, AABBCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });

    const auto forward = CollidePair(orientedBody, oriented, axisBody, axis);
    const auto reverse = CollidePair(axisBody, axis, orientedBody, oriented);
    REQUIRE(forward.has_value());
    REQUIRE(reverse.has_value());
    REQUIRE_FALSE(forward->Points.empty());
    REQUIRE_FALSE(reverse->Points.empty());
    CHECK(forward->Points.size() <= 4u);
    CHECK(reverse->Points.size() <= 4u);
    CHECK(Dot(forward->Points.front().Normal, reverse->Points.front().Normal) < -0.95f);
}

TEST_CASE("Resting box manifolds retain multiple warm-started impulses",
    "[PhysicsEngine][Manifold][WarmStart]")
{
    PhysicsWorld world;
    // Keep the intentionally overlapping resting configuration persistent so
    // the second step exercises warm-start matching rather than separation.
    world.Settings.MaxPositionCorrection = 0.0f;
    const BodyID planeBody = world.CreateRigidBody(StaticBody());
    [[maybe_unused]] const ColliderID planeCollider =
        world.AddCollider(planeBody, PlaneCollider{ Vec3f::Up(), 0.0f });

    const BodyID boxBody = world.CreateRigidBody(
        DynamicBoxBody(Vec3f{ 0.0f, 0.48f, 0.0f }));
    [[maybe_unused]] const ColliderID boxCollider =
        world.AddCollider(boxBody, BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });

    world.Step(1.0f / 60.0f);
    REQUIRE_FALSE(world.Contacts().empty());
    REQUIRE(world.Contacts().front().Points.size() >= 2u);

    world.Step(1.0f / 60.0f);
    REQUIRE_FALSE(world.Contacts().empty());
    REQUIRE(world.Contacts().front().Points.size() >= 2u);

    std::size_t warmedPoints = 0u;
    for (const ContactPoint& point : world.Contacts().front().Points)
    {
        if (point.NormalImpulse > 1.0e-6f) ++warmedPoints;
    }
    CHECK(warmedPoints >= 2u);
}

TEST_CASE("Triangle mesh validation builds a SAH BVH and rejects invalid data",
    "[PhysicsEngine][TriangleMesh][Validation]")
{
    const Collider valid = MakeCollider(0, 0, FloorMesh());
    const auto& mesh = std::get<TriangleMeshCollider>(valid.Shape);
    REQUIRE(mesh.Triangles.size() == 2u);
    REQUIRE_FALSE(mesh.Acceleration.Empty());
    CHECK(mesh.Acceleration.IsValid());
    CHECK(mesh.Acceleration.Stats.PrimitiveCount == 2u);

    TriangleMeshCollider invalid = FloorMesh();
    invalid.Triangles[0][2] = 99u;
    REQUIRE_THROWS_AS(MakeCollider(0, 0, std::move(invalid)), std::invalid_argument);

    TriangleMeshCollider degenerate = FloorMesh();
    degenerate.Triangles[0] = { 0u, 0u, 1u };
    REQUIRE_THROWS_AS(
        MakeCollider(0, 0, std::move(degenerate)), std::invalid_argument);

    PhysicsWorld world;
    const BodyID dynamicBody = world.CreateRigidBody(DynamicBoxBody(Vec3f::Zero()));
    const ColliderID dynamicMesh = world.AddCollider(dynamicBody, FloorMesh());
    REQUIRE(world.IsValidCollider(dynamicMesh));
    const auto& dynamicMeshShape =
        std::get<TriangleMeshCollider>(world.Colliders().at(dynamicMesh).Shape);
    REQUIRE_FALSE(dynamicMeshShape.Acceleration.Empty());
    CHECK(dynamicMeshShape.Acceleration.IsValid());
}

TEST_CASE("Static triangle meshes collide query raycast sweep and debug consistently",
    "[PhysicsEngine][TriangleMesh][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    const BodyID floorBody = world.CreateRigidBody(StaticBody());
    const ColliderID floor = world.AddCollider(floorBody, FloorMesh());
    const BodyID sphereBody = world.CreateRigidBody(
        DynamicSphereBody(Vec3f{ 0.0f, 0.40f, 0.0f }));
    const ColliderID sphere = world.AddCollider(sphereBody, SphereCollider{ 0.5f });

    const auto direct = CollidePair(
        world.Bodies().at(floorBody), world.Colliders().at(floor),
        world.Bodies().at(sphereBody), world.Colliders().at(sphere));
    REQUIRE(direct.has_value());
    REQUIRE_FALSE(direct->Points.empty());
    CHECK(direct->Points[0].Normal.y > 0.9f);
    CHECK(direct->Points[0].PenetrationDepth > 0.05f);

    const auto overlap = world.QueryAABB(AABBf::FromCenterExtent(
        Vec3f::Zero(), Vec3f{ 0.5f, 0.1f, 0.5f }));
    CHECK(std::find(overlap.begin(), overlap.end(), floor) != overlap.end());

    const auto ray = world.Raycast(
        Vec3f{ 0.0f, 2.0f, 0.0f }, -Vec3f::UnitY(), 10.0f,
        0xFFFF'FFFFu, sphere);
    REQUIRE(ray.has_value());
    CHECK(ray->Collider == floor);
    CHECK(ray->Distance == Catch::Approx(2.0f).margin(1.0e-4f));
    CHECK(ray->Point.y == Catch::Approx(0.0f).margin(1.0e-4f));
    CHECK(ray->Normal.y > 0.9f);

    const auto sweep = world.SweepSphere(
        Vec3f{ 0.0f, 2.0f, 0.0f }, Vec3f{ 0.0f, -3.0f, 0.0f }, 0.25f,
        0xFFFF'FFFFu, sphere);
    REQUIRE(sweep.has_value());
    CHECK(sweep->Collider == floor);
    CHECK(sweep->TimeOfImpact == Catch::Approx(1.75f / 3.0f).margin(2.0e-3f));
    CHECK(sweep->Normal.y > 0.9f);

    const auto debug = world.DebugShapes();
    const auto found = std::find_if(debug.begin(), debug.end(),
        [&](const DebugShape& shape) { return shape.Collider == floor; });
    REQUIRE(found != debug.end());
    CHECK(found->Kind == DebugShapeKind::TriangleMesh);
    CHECK(found->Vertices.size() == 4u);
    CHECK(found->Faces.size() == 2u);
}

TEST_CASE("Convex hull validation normalizes closed topology and rejects invalid input",
    "[PhysicsEngine][Convex][Validation]")
{
    const Collider valid = MakeCollider(0, 0, CubeHull());
    const auto& hull = std::get<ConvexHullCollider>(valid.Shape);
    REQUIRE(hull.Vertices.size() == 8u);
    REQUIRE(hull.Faces.size() == 12u);

    ConvexHullCollider open = CubeHull();
    open.Faces.pop_back();
    REQUIRE_THROWS_AS(MakeCollider(0, 0, std::move(open)), std::invalid_argument);

    ConvexHullCollider invalidIndex = CubeHull();
    invalidIndex.Faces[0][0] = 99u;
    REQUIRE_THROWS_AS(
        MakeCollider(0, 0, std::move(invalidIndex)), std::invalid_argument);

    ConvexHullCollider concave = CubeHull();
    concave.Vertices[6] = Vec3f::Zero();
    REQUIRE_THROWS_AS(
        MakeCollider(0, 0, std::move(concave)), std::invalid_argument);

    ConvexHullCollider duplicateVertex = CubeHull();
    duplicateVertex.Vertices[7] = duplicateVertex.Vertices[6];
    REQUIRE_THROWS_AS(
        MakeCollider(0, 0, std::move(duplicateVertex)), std::invalid_argument);

    ConvexHullCollider unusedVertex = CubeHull();
    unusedVertex.Vertices.push_back(Vec3f::Zero());
    REQUIRE_THROWS_AS(
        MakeCollider(0, 0, std::move(unusedVertex)), std::invalid_argument);
}

TEST_CASE("GJK and EPA resolve convex penetration deterministically",
    "[PhysicsEngine][Convex][GJK][EPA]")
{
    const RigidBody hullBody =
        MakeRigidBody(0, StaticBody(Vec3f::Zero()));
    const RigidBody sphereBody =
        MakeRigidBody(1, DynamicSphereBody(Vec3f{ 0.75f, 0.0f, 0.0f }));
    const Collider hull = MakeCollider(0, 0, CubeHull());
    const Collider sphere = MakeCollider(1, 1, SphereCollider{ 0.5f });

    const auto first = CollideConvex(hullBody, hull, sphereBody, sphere);
    const auto second = CollideConvex(hullBody, hull, sphereBody, sphere);
    REQUIRE(first.has_value());
    REQUIRE(second.has_value());
    CHECK(first->Normal.x == Catch::Approx(1.0f).margin(2.0e-3f));
    CHECK(first->Depth == Catch::Approx(0.25f).margin(2.0e-3f));
    CHECK(first->Depth == Catch::Approx(second->Depth));
    CHECK(first->GJKIterations == second->GJKIterations);
    CHECK(first->EPAIterations == second->EPAIterations);

    const RigidBody separatedBody =
        MakeRigidBody(1, DynamicSphereBody(Vec3f{ 2.0f, 0.0f, 0.0f }));
    CHECK_FALSE(CollideConvex(
        hullBody, hull, separatedBody, sphere).has_value());

    RigidBodyDesc rotatedDesc = DynamicBoxBody(
        Vec3f{ 0.7f, 0.1f, 0.05f });
    rotatedDesc.State.Rotation = RotationAroundZ(0.35f);
    const RigidBody rotatedBody = MakeRigidBody(2, rotatedDesc);
    const Collider rotatedHull = MakeCollider(2, 2, CubeHull());
    const auto hullHull =
        CollideConvex(hullBody, hull, rotatedBody, rotatedHull);
    const auto reverseHullHull =
        CollideConvex(rotatedBody, rotatedHull, hullBody, hull);
    REQUIRE(hullHull.has_value());
    REQUIRE(reverseHullHull.has_value());
    CHECK(hullHull->Depth > 0.0f);
    CHECK(hullHull->Depth ==
        Catch::Approx(reverseHullHull->Depth).margin(2.0e-3f));
    CHECK(Dot(hullHull->Normal, reverseHullHull->Normal) < -0.99f);
}

TEST_CASE("Convex narrowphase supports planes pair symmetry and rotated hulls",
    "[PhysicsEngine][Convex][Narrowphase]")
{
    RigidBodyDesc rotatedDesc = DynamicBoxBody(
        Vec3f{ 0.0f, 0.35f, 0.0f });
    rotatedDesc.State.Rotation = RotationAroundZ(0.3f);
    const RigidBody hullBody = MakeRigidBody(0, rotatedDesc);
    const RigidBody planeBody = MakeRigidBody(1, StaticBody());
    const Collider hull = MakeCollider(0, 0, CubeHull());
    const Collider plane =
        MakeCollider(1, 1, PlaneCollider{ Vec3f::Up(), 0.0f });

    const auto hullPlane = CollidePair(hullBody, hull, planeBody, plane);
    const auto planeHull = CollidePair(planeBody, plane, hullBody, hull);
    REQUIRE(hullPlane.has_value());
    REQUIRE(planeHull.has_value());
    CHECK(hullPlane->Points[0].PenetrationDepth > 0.0f);
    CHECK(hullPlane->Points[0].Normal.y < 0.0f);
    CHECK(planeHull->Points[0].Normal.y > 0.0f);
}

TEST_CASE("Convex colliders participate in world solving rays sweeps and debug",
    "[PhysicsEngine][Convex][World]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    const BodyID hullBody = world.CreateRigidBody(StaticBody());
    const BodyID sphereBody = world.CreateRigidBody(
        DynamicSphereBody(Vec3f{ 0.75f, 0.0f, 0.0f }));
    const ColliderID hullCollider =
        world.AddCollider(hullBody, CubeHull());
    [[maybe_unused]] const ColliderID sphereCollider =
        world.AddCollider(sphereBody, SphereCollider{ 0.5f });

    world.Step(1.0f / 60.0f);
    REQUIRE(!world.Contacts().empty());

    const auto ray = world.Raycast(
        Vec3f{ -2.0f, 0.0f, 0.0f }, Vec3f::UnitX(), 5.0f);
    REQUIRE(ray.has_value());
    CHECK(ray->Collider == hullCollider);
    CHECK(ray->Distance == Catch::Approx(1.5f).margin(1.0e-4f));
    CHECK(ray->Normal.x == Catch::Approx(-1.0f).margin(1.0e-4f));

    const auto sweep = world.SweepSphere(
        Vec3f{ -2.0f, 0.0f, 0.0f }, Vec3f{ 1.0f, 0.0f, 0.0f }, 0.25f);
    REQUIRE_FALSE(sweep.has_value());
    const auto impact = world.SweepSphere(
        Vec3f{ -2.0f, 0.0f, 0.0f }, Vec3f{ 4.0f, 0.0f, 0.0f }, 0.25f);
    REQUIRE(impact.has_value());
    CHECK(impact->Collider == hullCollider);
    CHECK(impact->Distance == Catch::Approx(1.25f).margin(2.0e-3f));

    const auto shapes = CollectDebugShapes(world.Bodies(), world.Colliders());
    REQUIRE(shapes.size() == 2u);
    CHECK(shapes[0].Kind == DebugShapeKind::ConvexHull);
    CHECK(shapes[0].Vertices.size() == 8u);
    CHECK(shapes[0].Faces.size() == 12u);
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

TEST_CASE("Capsule colliders provide bounds and contact all V1 rigid shapes", "[PhysicsEngine][Narrowphase]")
{
    const RigidBody capsuleBody =
        MakeRigidBody(0, DynamicSphereBody(Vec3f{ 0.0f, 0.4f, 0.0f }));
    const RigidBody sphereBody =
        MakeRigidBody(1, DynamicSphereBody(Vec3f{ 0.8f, 0.4f, 0.0f }));
    const RigidBody planeBody =
        MakeRigidBody(2, StaticBody());
    const RigidBody boxBody =
        MakeRigidBody(3, StaticBody(Vec3f{ 0.6f, 0.4f, 0.0f }));

    const Collider capsule =
        MakeCollider(0, 0, CapsuleCollider{ 0.5f, 0.6f });
    const Collider sphere =
        MakeCollider(1, 1, SphereCollider{ 0.5f });
    const Collider plane =
        MakeCollider(2, 2, PlaneCollider{ Vec3f::Up(), 0.0f });
    const Collider box =
        MakeCollider(3, 3, BoxCollider{ Vec3f{ 0.35f, 0.35f, 0.35f } });

    const AABBf bounds = WorldAABB(capsuleBody, capsule);
    REQUIRE(bounds.Min.y == Catch::Approx(-0.7f));
    REQUIRE(bounds.Max.y == Catch::Approx(1.5f));

    const auto capsuleSphere = CollidePair(capsuleBody, capsule, sphereBody, sphere);
    const auto capsulePlane = CollidePair(capsuleBody, capsule, planeBody, plane);
    const auto capsuleBox = CollidePair(capsuleBody, capsule, boxBody, box);
    const auto sphereCapsule = CollidePair(sphereBody, sphere, capsuleBody, capsule);

    REQUIRE(capsuleSphere.has_value());
    REQUIRE(capsulePlane.has_value());
    REQUIRE(capsuleBox.has_value());
    REQUIRE(sphereCapsule.has_value());
    REQUIRE(capsuleSphere->Points.front().Normal.x == Catch::Approx(1.0f));
    REQUIRE(sphereCapsule->Points.front().Normal.x == Catch::Approx(-1.0f));
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

TEST_CASE("World debug shape extraction mirrors active collider geometry", "[PhysicsEngine][Debug]")
{
    PhysicsWorld world;

    const BodyID dynamic =
        world.CreateRigidBody(DynamicSphereBody(Vec3f{ 1.0f, 2.0f, 3.0f }));
    const BodyID staticBody =
        world.CreateRigidBody(StaticBody());

    [[maybe_unused]] const ColliderID sphere =
        world.AddCollider(dynamic, SphereCollider{ 0.25f });
    [[maybe_unused]] const ColliderID capsule =
        world.AddCollider(dynamic, CapsuleCollider{ 0.5f, 0.75f });
    [[maybe_unused]] const ColliderID box =
        world.AddCollider(staticBody, BoxCollider{ Vec3f{ 1.0f, 2.0f, 3.0f } });
    [[maybe_unused]] const ColliderID plane =
        world.AddCollider(staticBody, PlaneCollider{ Vec3f::Up(), -1.0f });

    const std::vector<DebugShape> shapes = world.DebugShapes();
    REQUIRE(shapes.size() == 4u);
    REQUIRE(shapes[0].Kind == DebugShapeKind::Sphere);
    REQUIRE(shapes[0].Radius == Catch::Approx(0.25f));
    REQUIRE(shapes[1].Kind == DebugShapeKind::Capsule);
    REQUIRE(shapes[1].SegmentEnd.y - shapes[1].SegmentStart.y == Catch::Approx(1.5f));
    REQUIRE(shapes[2].Kind == DebugShapeKind::Box);
    REQUIRE(shapes[2].HalfExtents.z == Catch::Approx(3.0f));
    REQUIRE(shapes[3].Kind == DebugShapeKind::Plane);
    REQUIRE(shapes[3].PlaneDistance == Catch::Approx(-1.0f));
}

TEST_CASE("Projectile system handles hitscan ballistic owner ignore and trigger hits", "[PhysicsEngine][Projectile]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();

    const BodyID ownerBody = world.CreateRigidBody(StaticBody(Vec3f::Zero()));
    const BodyID wallBody = world.CreateRigidBody(StaticBody(Vec3f{ 3.0f, 0.0f, 0.0f }));
    const BodyID triggerBody = world.CreateRigidBody(StaticBody(Vec3f{ 5.0f, 0.0f, 0.0f }));
    const ColliderID owner = world.AddCollider(ownerBody, SphereCollider{ 0.5f });
    const ColliderID wall = world.AddCollider(wallBody, AABBCollider{ Vec3f{ 0.25f, 1.0f, 1.0f } });
    const ColliderID trigger = world.AddCollider(triggerBody, SphereCollider{ 0.5f });
    world.SetColliderTrigger(trigger, true);

    ProjectileSystem projectiles;
    std::vector<ProjectileHitEvent> callbacks;
    projectiles.SetHitCallback([&callbacks](const ProjectileHitEvent& event) { callbacks.push_back(event); });

    ProjectileDesc hitscan;
    hitscan.Mode = ProjectileMode::Hitscan;
    hitscan.Position = Vec3f::Zero();
    hitscan.Velocity = Vec3f{ 10.0f, 0.0f, 0.0f };
    hitscan.MaxDistance = 10.0f;
    hitscan.IgnoredOwnerCollider = owner;
    const ProjectileID ray = projectiles.Spawn(hitscan);
    projectiles.Step(world, 1.0f / 60.0f);
    REQUIRE_FALSE(projectiles.IsActive(ray));
    REQUIRE(projectiles.LastHits().size() == 1u);
    REQUIRE(projectiles.LastHits().front().Sweep.Collider == wall);

    ProjectileDesc ballistic = hitscan;
    ballistic.Mode = ProjectileMode::Ballistic;
    ballistic.Position = Vec3f{ 3.6f, 0.0f, 0.0f };
    ballistic.Radius = 0.1f;
    ballistic.Response = ProjectileImpactResponse::Pierce;
    ballistic.Lifetime = 1.0f;
    ballistic.MaxDistance = 10.0f;
    const ProjectileID ball = projectiles.Spawn(ballistic);
    projectiles.Step(world, 0.2f);
    REQUIRE(projectiles.IsActive(ball));
    REQUIRE(projectiles.LastHits().size() == 1u);
    REQUIRE(projectiles.LastHits().front().Sweep.Collider == trigger);
    REQUIRE(projectiles.LastHits().front().IsTrigger);
    REQUIRE(callbacks.size() == 2u);

    ProjectileDesc invalid = hitscan;
    invalid.Radius = 0.0f;
    REQUIRE_THROWS_AS(projectiles.Spawn(invalid), std::invalid_argument);
}

TEST_CASE("Buoyancy volumes apply submersion forces and lifecycle events", "[PhysicsEngine][Buoyancy]")
{
    PhysicsWorld world;
    world.Settings.EnableSleeping = false;

    RigidBodyDesc desc = DynamicSphereBody(Vec3f{ 0.0f, 0.0f, 0.0f }, 0.5f);
    desc.EnableGravity = false;
    desc.State.LinearVelocity = Vec3f{ 2.0f, 0.0f, 0.0f };
    const BodyID body = world.CreateRigidBody(desc);
    [[maybe_unused]] const ColliderID collider = world.AddCollider(body, SphereCollider{ 0.5f });

    BuoyancySystem water;
    WaterVolumeDesc volume;
    volume.Bounds = AABBf::FromMinMax(Vec3f{ -2.0f, -1.0f, -2.0f }, Vec3f{ 2.0f, 1.0f, 2.0f });
    volume.Density = 1.0f;
    volume.LinearDrag = 2.0f;
    const WaterVolumeID volumeID = water.AddWaterVolume(volume);
    water.RegisterBody(body, BuoyancyBodyDesc{ 1.0f, 1.0f, 1.0f, 1.0f });

    water.Step(world, 1.0f / 60.0f);
    REQUIRE(water.LastEvents().size() == 1u);
    REQUIRE(water.LastEvents().front().Volume == volumeID);
    REQUIRE(water.LastEvents().front().Type == WaterVolumeEventType::Enter);
    world.Step(1.0f / 60.0f);
    REQUIRE(world.Bodies()[body].State.LinearVelocity.y > 0.0f);
    REQUIRE(world.Bodies()[body].State.LinearVelocity.x < 2.0f);

    water.Step(world, 1.0f / 60.0f);
    REQUIRE(water.LastEvents().front().Type == WaterVolumeEventType::Stay);

    world.Bodies()[body].State.Position = Vec3f{ 10.0f, 0.0f, 0.0f };
    water.Step(world, 1.0f / 60.0f);
    REQUIRE(water.LastEvents().size() == 1u);
    REQUIRE(water.LastEvents().front().Type == WaterVolumeEventType::Exit);

    WaterVolumeDesc invalid = volume;
    invalid.Density = 0.0f;
    REQUIRE_THROWS_AS(water.AddWaterVolume(invalid), std::invalid_argument);
}

TEST_CASE("World swept spheres report deterministic continuous impacts", "[PhysicsEngine][World][Sweeps]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();

    const BodyID sphereBody =
        world.CreateRigidBody(StaticBody(Vec3f{ 3.0f, 1.0f, 0.0f }));
    const BodyID capsuleBody =
        world.CreateRigidBody(StaticBody(Vec3f{ 6.0f, 1.0f, 0.0f }));
    const BodyID floorBody =
        world.CreateRigidBody(StaticBody());

    const ColliderID sphere =
        world.AddCollider(sphereBody, SphereCollider{ 0.5f });
    const ColliderID capsule =
        world.AddCollider(capsuleBody, CapsuleCollider{ 0.5f, 0.75f });
    [[maybe_unused]] const ColliderID floor =
        world.AddCollider(floorBody, PlaneCollider{ Vec3f::Up(), 0.0f });

    const auto sphereHit =
        world.SweepSphere(Vec3f{ 0.0f, 1.0f, 0.0f }, Vec3f{ 5.0f, 0.0f, 0.0f }, 0.25f);
    REQUIRE(sphereHit.has_value());
    REQUIRE(sphereHit->Collider == sphere);
    REQUIRE(sphereHit->Distance == Catch::Approx(2.25f).margin(1.0e-3f));
    REQUIRE(sphereHit->Normal.x == Catch::Approx(-1.0f).margin(1.0e-3f));

    const auto capsuleRay =
        world.Raycast(Vec3f{ 4.0f, 1.0f, 0.0f }, Vec3f::UnitX(), 4.0f);
    REQUIRE(capsuleRay.has_value());
    REQUIRE(capsuleRay->Collider == capsule);

    const auto planeHit =
        world.SweepSphere(Vec3f{ 0.0f, 2.0f, 0.0f }, Vec3f{ 0.0f, -3.0f, 0.0f }, 0.5f);
    REQUIRE(planeHit.has_value());
    REQUIRE(planeHit->Distance == Catch::Approx(1.5f).margin(1.0e-3f));
    REQUIRE(planeHit->Normal.y == Catch::Approx(1.0f).margin(1.0e-3f));

    REQUIRE_THROWS_AS(
        world.SweepSphere(Vec3f::Zero(), Vec3f::Zero(), 0.5f),
        std::invalid_argument);
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
    REQUIRE_FALSE(contact->Points.empty());
    CHECK(contact->Points.size() <= 4u);
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


TEST_CASE("Dynamic triangle meshes collide with convex peers", "[PhysicsEngine][TriangleMesh][Dynamic]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    RigidBodyDesc moving = DynamicBoxBody(Vec3f::Zero(), Vec3f{ 1.0f, 0.1f, 1.0f });
    moving.EnableGravity = false;
    moving.State.LinearVelocity = Vec3f{ 0.0f, 1.0f, 0.0f };
    const BodyID meshBody = world.CreateRigidBody(moving);
    const ColliderID meshCollider = world.AddCollider(meshBody, FloorMesh(1.0f));

    const BodyID sphereBody = world.CreateRigidBody(StaticBody(Vec3f{ 0.0f, 0.1f, 0.0f }));
    const ColliderID sphereCollider = world.AddCollider(sphereBody, SphereCollider{ 0.25f });

    world.Step(1.0f / 60.0f);

    REQUIRE(world.IsValidCollider(meshCollider));
    REQUIRE(world.IsValidCollider(sphereCollider));
    REQUIRE_FALSE(world.Contacts().empty());
    const bool expectedPair =
        (world.Contacts().front().ColliderA == meshCollider &&
         world.Contacts().front().ColliderB == sphereCollider) ||
        (world.Contacts().front().ColliderA == sphereCollider &&
         world.Contacts().front().ColliderB == meshCollider);
    CHECK(expectedPair);
}

TEST_CASE("Kinematic triangle meshes track body transforms", "[PhysicsEngine][TriangleMesh][Kinematic]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    RigidBodyDesc moving;
    moving.Type = BodyType::Kinematic;
    moving.State.Position = Vec3f{ 0.0f, -1.0f, 0.0f };
    moving.State.LinearVelocity = Vec3f{ 0.0f, 2.0f, 0.0f };
    moving.Mass = StaticMassProperties();
    const BodyID meshBody = world.CreateRigidBody(moving);
    const ColliderID meshCollider = world.AddCollider(meshBody, FloorMesh(1.0f));

    const BodyID sphereBody = world.CreateRigidBody(StaticBody(Vec3f{ 0.0f, 0.15f, 0.0f }));
    [[maybe_unused]] const ColliderID sphereCollider =
        world.AddCollider(sphereBody, SphereCollider{ 0.25f });

    for (int step = 0; step < 30 && world.Contacts().empty(); ++step)
        world.Step(1.0f / 60.0f);

    REQUIRE(world.IsValidCollider(meshCollider));
    REQUIRE_FALSE(world.Contacts().empty());
}

TEST_CASE("Triangle mesh pairs use BVH accelerated prism contacts", "[PhysicsEngine][TriangleMesh][MeshMesh]")
{
    const RigidBody bodyA = MakeRigidBody(0, StaticBody(Vec3f::Zero()));
    const RigidBody bodyB = MakeRigidBody(1, StaticBody(Vec3f{ 0.0f, 5.0e-4f, 0.0f }));
    const Collider colliderA = MakeCollider(0, 0, FloorMesh(1.0f));
    const Collider colliderB = MakeCollider(
        1, 1, FloorMesh(1.0f), {}, Vec3f::Zero(), RotationAroundZ(0.05f));

    const auto contact = CollidePair(bodyA, colliderA, bodyB, colliderB);
    REQUIRE(contact.has_value());
    REQUIRE_FALSE(contact->Points.empty());
    CHECK(contact->Points.front().PenetrationDepth > 0.0f);
}

TEST_CASE("Continuous dynamic triangle meshes receive conservative CCD clipping", "[PhysicsEngine][TriangleMesh][CCD]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    world.Settings.EnableSleeping = false;

    RigidBodyDesc moving = DynamicBoxBody(
        Vec3f{ -4.0f, 0.0f, 0.0f }, Vec3f{ 0.5f, 0.1f, 0.5f });
    moving.EnableGravity = false;
    moving.CollisionDetection = CollisionDetectionMode::Continuous;
    moving.State.LinearVelocity = Vec3f{ 40.0f, 0.0f, 0.0f };
    const BodyID meshBody = world.CreateRigidBody(moving);
    [[maybe_unused]] const ColliderID meshCollider =
        world.AddCollider(meshBody, FloorMesh(0.5f));

    const BodyID obstacleBody = world.CreateRigidBody(StaticBody(Vec3f::Zero()));
    [[maybe_unused]] const ColliderID obstacleCollider =
        world.AddCollider(obstacleBody, SphereCollider{ 0.5f });

    world.Step(0.2f);

    CHECK(world.Bodies().at(meshBody).State.Position.x < 1.0f);
}

TEST_CASE("Physics snapshots round-trip deterministic world state",
    "[PhysicsEngine][Serialization]")
{
    PhysicsWorld world;
    world.Settings.EnableSleeping = false;
    world.Settings.EnableParallelIslands = true;
    world.Gravity = Vec3f{ 0.0f, -9.5f, 0.0f };

    const BodyID floor = world.CreateRigidBody(StaticBody());
    const ColliderID floorCollider = world.AddCollider(floor, FloorMesh());
    const BodyID box = world.CreateRigidBody(DynamicBoxBody(Vec3f{ 0.0f, 1.0f, 0.0f }));
    const ColliderID boxCollider = world.AddCollider(box, BoxCollider{});
    world.SetCollisionFilter(boxCollider, CollisionLayer::DynamicWorld,
        CollisionLayer::StaticWorld | CollisionLayer::DynamicWorld);
    world.SetCollisionFilter(floorCollider, CollisionLayer::StaticWorld, CollisionLayer::DynamicWorld);
    world.SetBodyCollisionDetectionMode(box, CollisionDetectionMode::Continuous);
    [[maybe_unused]] const JointID tether = world.CreateDistanceJoint(
        floor, box, Vec3f::Zero(), Vec3f{ 0.0f, 1.0f, 0.0f }, 1.0f, true);
    world.SetCollisionPairResponse(floorCollider, boxCollider, CollisionResponse::Block);
    world.AddBodyForce(box, Vec3f{ 3.0f, 0.0f, 0.0f });
    [[maybe_unused]] const auto fixedSteps = world.StepFixed(0.013f, 1.0f / 120.0f, 8u);

    const std::uint64_t before = PhysicsStateHash(world);
    const auto bytes = SerializePhysicsWorldSnapshot(world.CaptureSnapshot());
    REQUIRE_FALSE(bytes.empty());

    PhysicsWorld restored;
    restored.RestoreSnapshot(DeserializePhysicsWorldSnapshot(bytes));
    CHECK(PhysicsStateHash(restored) == before);
    CHECK(restored.Bodies().size() == world.Bodies().size());
    CHECK(restored.Colliders().size() == world.Colliders().size());
    CHECK(restored.Joints().size() == world.Joints().size());
    CHECK(restored.FixedAccumulator() == Catch::Approx(world.FixedAccumulator()));
    REQUIRE(std::holds_alternative<TriangleMeshCollider>(restored.Colliders().at(floorCollider).Shape));
    CHECK_FALSE(std::get<TriangleMeshCollider>(restored.Colliders().at(floorCollider).Shape).Acceleration.Empty());

    restored.Step(1.0f / 120.0f);
    world.Step(1.0f / 120.0f);
    CHECK(PhysicsStateHash(restored) == PhysicsStateHash(world));
}

TEST_CASE("Physics snapshot file load is strong-exception safe",
    "[PhysicsEngine][Serialization][File]")
{
    PhysicsWorld world;
    world.Gravity = Vec3f::Zero();
    const BodyID body = world.CreateRigidBody(DynamicSphereBody(Vec3f{ 1.0f, 2.0f, 3.0f }));
    world.AddCollider(body, SphereCollider{ 0.5f });
    const std::uint64_t expected = PhysicsStateHash(world);

    const auto path = std::filesystem::temp_directory_path() / "kairo-physics-snapshot-test.kphys";
    SavePhysicsWorld(path, world);
    PhysicsWorld loaded;
    LoadPhysicsWorld(path, loaded);
    CHECK(PhysicsStateHash(loaded) == expected);

    { std::ofstream corrupt(path, std::ios::binary | std::ios::trunc); corrupt << "bad"; }
    REQUIRE_THROWS(LoadPhysicsWorld(path, loaded));
    CHECK(PhysicsStateHash(loaded) == expected);
    std::error_code ec; std::filesystem::remove(path, ec);
}

TEST_CASE("Physics replay reproduces command-driven simulation and detects divergence",
    "[PhysicsEngine][Replay]")
{
    PhysicsWorld recordedWorld;
    recordedWorld.Gravity = Vec3f::Zero();
    recordedWorld.Settings.EnableSleeping = false;
    const BodyID body = recordedWorld.CreateRigidBody(DynamicSphereBody(Vec3f::Zero(), 0.25f));
    recordedWorld.AddCollider(body, SphereCollider{ 0.25f });

    PhysicsReplayRecorder recorder(recordedWorld, 1.0f / 120.0f);
    for (int frame = 0; frame < 40; ++frame)
    {
        std::vector<PhysicsReplayCommand> commands;
        if (frame < 10) commands.push_back(ReplayAddForce{ body, Vec3f{ 2.0f, 0.5f, 0.0f } });
        if (frame == 12) commands.push_back(ReplayImpulseAtPoint{ body, Vec3f{ 0.0f, 1.0f, 0.0f }, Vec3f::Zero() });
        recorder.Step(recordedWorld, std::move(commands));
    }

    PhysicsReplay replay = std::move(recorder).TakeReplay();
    const auto replayBytes = SerializePhysicsReplay(replay);
    const PhysicsReplay parsed = DeserializePhysicsReplay(replayBytes);
    PhysicsWorld playback;
    const PhysicsReplayVerification verified = VerifyPhysicsReplay(parsed, playback);
    CHECK(verified.Matched);
    CHECK(verified.VerifiedFrames == 40u);
    CHECK(PhysicsStateHash(playback) == PhysicsStateHash(recordedWorld));

    PhysicsReplay divergent = parsed;
    REQUIRE(divergent.Frames.size() > 5u);
    divergent.Frames[5].ExpectedStateHash ^= 0x1ull;
    PhysicsWorld divergenceWorld;
    const PhysicsReplayVerification mismatch = VerifyPhysicsReplay(divergent, divergenceWorld);
    CHECK_FALSE(mismatch.Matched);
    CHECK(mismatch.DivergentFrame == 5u);
    CHECK(mismatch.ExpectedHash != mismatch.ActualHash);
}

TEST_CASE("Physics snapshot and replay parsers reject truncation and trailing data",
    "[PhysicsEngine][Serialization][Validation]")
{
    PhysicsWorld world;
    const auto snapshot = SerializePhysicsWorldSnapshot(world.CaptureSnapshot());
    REQUIRE(snapshot.size() > 8u);
    auto truncated = snapshot;
    truncated.pop_back();
    REQUIRE_THROWS(DeserializePhysicsWorldSnapshot(truncated));
    auto trailing = snapshot;
    trailing.push_back(0xffu);
    REQUIRE_THROWS(DeserializePhysicsWorldSnapshot(trailing));

    PhysicsReplay replay;
    replay.InitialState = world.CaptureSnapshot();
    const auto replayBytes = SerializePhysicsReplay(replay);
    auto badReplay = replayBytes;
    badReplay.pop_back();
    REQUIRE_THROWS(DeserializePhysicsReplay(badReplay));
}
