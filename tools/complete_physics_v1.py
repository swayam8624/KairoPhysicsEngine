from pathlib import Path

world = Path("PhysicsWorld.cppm")
text = world.read_text()
guard = '''            if (std::holds_alternative<TriangleMeshCollider>(shape) &&
                m_Bodies.at(body).Type != BodyType::Static)
                throw std::invalid_argument(
                    "TriangleMeshCollider is static-only until concave dynamic manifold support lands.");

'''
if guard not in text:
    raise SystemExit("triangle mesh static-only guard not found")
text = text.replace(guard, "", 1)

radius_old = '''            // Infinite planes cannot be swept as moving source volumes and
            // triangle meshes are intentionally static-only in this engine.
            return 0.0f;
'''
radius_new = '''            if (const auto* mesh = std::get_if<TriangleMeshCollider>(&collider.Shape))
            {
                float radius = 0.0f;
                for (const Vec3f& vertex : mesh->Vertices)
                {
                    radius = std::max(radius, vertex.Length());
                }
                return radius;
            }

            // Infinite planes cannot be swept as moving source volumes.
            return 0.0f;
'''
if radius_old not in text:
    raise SystemExit("triangle mesh CCD radius marker not found")
text = text.replace(radius_old, radius_new, 1)
world.write_text(text)

narrow = Path("Narrowphase.cppm")
text = narrow.read_text()
marker = '''    [[nodiscard]]
    inline std::optional<ContactManifold> CollideTriangleMesh(
'''
if marker not in text:
    raise SystemExit("triangle mesh narrowphase marker not found")
mesh_mesh = '''    [[nodiscard]]
    inline std::optional<ContactManifold> CollideTriangleMeshes(
        const RigidBody& bodyA,
        const Collider& colliderA,
        const TriangleMeshCollider& meshA,
        const RigidBody& bodyB,
        const Collider& colliderB,
        const TriangleMeshCollider& meshB)
    {
        if (meshA.Acceleration.Empty() || meshB.Acceleration.Empty())
            return std::nullopt;

        const AABBf boundsB = WorldAABB(bodyB, colliderB);
        if (!boundsB.IsValid()) return std::nullopt;

        const AABBf queryA =
            WorldBoundsToColliderLocal(bodyA, colliderA, boundsB);
        const auto candidatesA = kairo::foundation::spatial::QueryAABB(
            meshA.Acceleration, queryA);

        std::optional<ConvexPenetration> deepest;
        for (const auto triangleA : candidatesA.PrimitiveIndices)
        {
            if (triangleA >= meshA.Triangles.size()) continue;

            Collider prismA = colliderA;
            prismA.Shape = MakeTriangleCollisionPrism(meshA, triangleA);
            const AABBf prismABounds = WorldAABB(bodyA, prismA);
            if (!prismABounds.IsValid()) continue;

            const AABBf queryB =
                WorldBoundsToColliderLocal(bodyB, colliderB, prismABounds);
            const auto candidatesB = kairo::foundation::spatial::QueryAABB(
                meshB.Acceleration, queryB);

            for (const auto triangleB : candidatesB.PrimitiveIndices)
            {
                if (triangleB >= meshB.Triangles.size()) continue;

                Collider prismB = colliderB;
                prismB.Shape = MakeTriangleCollisionPrism(meshB, triangleB);
                const auto penetration =
                    CollideConvex(bodyA, prismA, bodyB, prismB);
                if (penetration &&
                    (!deepest || penetration->Depth > deepest->Depth))
                {
                    deepest = penetration;
                }
            }
        }

        if (!deepest) return std::nullopt;

        ContactManifold manifold = MakeContactManifold(
            bodyA.ID, bodyB.ID, colliderA.ID, colliderB.ID,
            colliderA.IsTrigger || colliderB.IsTrigger);
        manifold.Points.push_back(MakeContactPoint(
            deepest->Position, deepest->Normal, deepest->Depth));
        return manifold;
    }

'''
text = text.replace(marker, mesh_mesh + marker, 1)

old_dispatch = '''        if (const auto* meshA = std::get_if<TriangleMeshCollider>(&colliderA.Shape))
        {
            return CollideTriangleMesh(
                bodyA, colliderA, *meshA, bodyB, colliderB);
        }
'''
new_dispatch = '''        if (const auto* meshA = std::get_if<TriangleMeshCollider>(&colliderA.Shape))
        {
            if (const auto* meshB = std::get_if<TriangleMeshCollider>(&colliderB.Shape))
            {
                return CollideTriangleMeshes(
                    bodyA, colliderA, *meshA, bodyB, colliderB, *meshB);
            }

            return CollideTriangleMesh(
                bodyA, colliderA, *meshA, bodyB, colliderB);
        }
'''
if old_dispatch not in text:
    raise SystemExit("triangle mesh dispatch marker not found")
text = text.replace(old_dispatch, new_dispatch, 1)
narrow.write_text(text)

tests = Path("tests/PhysicsEngineTests.cpp")
text = tests.read_text()
test_marker = '''TEST_CASE("Physics snapshots round-trip deterministic world state",
'''
if test_marker not in text:
    raise SystemExit("test insertion marker not found")
additions = r'''TEST_CASE("Dynamic triangle meshes collide with convex peers", "[PhysicsEngine][TriangleMesh][Dynamic]")
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
    CHECK((world.Contacts().front().ColliderA == meshCollider &&
           world.Contacts().front().ColliderB == sphereCollider) ||
          (world.Contacts().front().ColliderA == sphereCollider &&
           world.Contacts().front().ColliderB == meshCollider));
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

'''
text = text.replace(test_marker, additions + test_marker, 1)
tests.write_text(text)

readme = Path("README.md")
text = readme.read_text()
replacements = {
    "Sphere, plane, AABB, oriented box, validated convex-hull, and static triangle-mesh shapes":
        "Sphere, plane, AABB, oriented box, validated convex-hull, and rigid triangle-mesh shapes",
    "Static triangle meshes with KairoSpatial SAH BVH acceleration, exact rays/sweeps, and convex contact generation":
        "Rigid triangle meshes on static, kinematic, and dynamic bodies with KairoSpatial SAH BVH acceleration, exact rays/sweeps, convex contacts, and mesh-pair contacts",
    "Dynamic/kinematic concave triangle-mesh bodies (static triangle meshes are implemented)\nFull editor/ImGui tooling":
        "Full editor/ImGui tooling",
    "This is deliberately conservative for boxes/capsules/hulls: their enclosing sweep\nsphere can stop slightly early, but it cannot miss a translation-only tunnel that\nthe bound covers. Angular swept-volume CCD remains a future refinement.":
        "This is deliberately conservative for boxes/capsules/hulls/triangle meshes: their enclosing sweep\nsphere can stop slightly early, but it cannot miss a translation-only tunnel that\nthe bound covers. Angular swept-volume CCD remains a future refinement."
}
for old, new in replacements.items():
    if old not in text:
        raise SystemExit(f"README marker not found: {old[:60]}")
    text = text.replace(old, new, 1)
readme.write_text(text)
