from pathlib import Path

path = Path("tests/PhysicsEngineTests.cpp")
text = path.read_text()
old = '''    CHECK((world.Contacts().front().ColliderA == meshCollider &&
           world.Contacts().front().ColliderB == sphereCollider) ||
          (world.Contacts().front().ColliderA == sphereCollider &&
           world.Contacts().front().ColliderB == meshCollider));
'''
new = '''    const bool expectedPair =
        (world.Contacts().front().ColliderA == meshCollider &&
         world.Contacts().front().ColliderB == sphereCollider) ||
        (world.Contacts().front().ColliderA == sphereCollider &&
         world.Contacts().front().ColliderB == meshCollider);
    CHECK(expectedPair);
'''
if old not in text:
    raise SystemExit("expected Catch2 assertion not found")
path.write_text(text.replace(old, new, 1))
