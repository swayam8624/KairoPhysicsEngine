from pathlib import Path

path = Path("tests/PhysicsEngineTests.cpp")
text = path.read_text()
old = '''    PhysicsWorld world;
    const BodyID dynamicBody = world.CreateRigidBody(DynamicBoxBody(Vec3f::Zero()));
    REQUIRE_THROWS_AS(
        world.AddCollider(dynamicBody, FloorMesh()), std::invalid_argument);
}
'''
new = '''    PhysicsWorld world;
    const BodyID dynamicBody = world.CreateRigidBody(DynamicBoxBody(Vec3f::Zero()));
    const ColliderID dynamicMesh = world.AddCollider(dynamicBody, FloorMesh());
    REQUIRE(world.IsValidCollider(dynamicMesh));
    const auto& dynamicMeshShape =
        std::get<TriangleMeshCollider>(world.Colliders().at(dynamicMesh).Shape);
    REQUIRE_FALSE(dynamicMeshShape.Acceleration.Empty());
    CHECK(dynamicMeshShape.Acceleration.IsValid());
}
'''
if old not in text:
    raise SystemExit("legacy dynamic mesh rejection assertion not found")
path.write_text(text.replace(old, new, 1))
