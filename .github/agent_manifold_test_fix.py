from pathlib import Path

p = Path('tests/PhysicsEngineTests.cpp')
s = p.read_text()

old = '''    PhysicsWorld world;
    const BodyID planeBody = world.CreateRigidBody(StaticBody());
    world.AddCollider(planeBody, PlaneCollider{ Vec3f::Up(), 0.0f });

    const BodyID boxBody = world.CreateRigidBody(
        DynamicBoxBody(Vec3f{ 0.0f, 0.48f, 0.0f }));
    world.AddCollider(boxBody, BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });'''
new = '''    PhysicsWorld world;
    // Keep the intentionally overlapping resting configuration persistent so
    // the second step exercises warm-start matching rather than separation.
    world.Settings.MaxPositionCorrection = 0.0f;
    const BodyID planeBody = world.CreateRigidBody(StaticBody());
    [[maybe_unused]] const ColliderID planeCollider =
        world.AddCollider(planeBody, PlaneCollider{ Vec3f::Up(), 0.0f });

    const BodyID boxBody = world.CreateRigidBody(
        DynamicBoxBody(Vec3f{ 0.0f, 0.48f, 0.0f }));
    [[maybe_unused]] const ColliderID boxCollider =
        world.AddCollider(boxBody, BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } });'''
if old not in s:
    raise SystemExit('warm-start setup snippet not found')
s = s.replace(old, new, 1)

old = '''    REQUIRE(contact.has_value());
    REQUIRE(contact->Points.size() == 1);
    REQUIRE(contact->Points[0].PenetrationDepth > 0.0f);
}

TEST_CASE("Rotated BoxCollider contacts planes through projected radius"'''
new = '''    REQUIRE(contact.has_value());
    REQUIRE_FALSE(contact->Points.empty());
    CHECK(contact->Points.size() <= 4u);
    REQUIRE(contact->Points[0].PenetrationDepth > 0.0f);
}

TEST_CASE("Rotated BoxCollider contacts planes through projected radius"'''
if old not in s:
    raise SystemExit('legacy rotated-box point-count snippet not found')
s = s.replace(old, new, 1)
p.write_text(s)
