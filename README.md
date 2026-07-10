# KairoPhysicsEngine

`KairoPhysicsEngine` is the rigid-body simulation layer built on top of:

```text
KairoMath -> KairoPhysicsMath -> KairoGeometry -> KairoSpatial -> KairoPhysicsEngine
```

This repo owns simulation concepts such as rigid bodies, colliders, broadphase
pairs, narrowphase contacts, contact solving, stepping, and debug extraction.
It is not a renderer, editor, ImGui tool, or sandbox viewer.

## Build

```bash
cd /Users/swayamsingal/Desktop/Programming/Kairo/Foundation/KairoPhysicsEngine
cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++
cmake --build build
ctest --test-dir build --output-on-failure
```

Dependencies resolve local Kairo repos first:

```text
KAIRO_PHYSICS_MATH_SOURCE_DIR -> ../KairoPhysicsMath -> GitHub
KAIRO_SPATIAL_SOURCE_DIR      -> ../Spatial          -> GitHub
```

## Implemented V1 Surface

Current V1 is a deterministic CPU rigid-body engine core:

```text
RigidBody records with MotionState, MassProperties, and ForceAccumulation
Sphere, plane, and AABB colliders
Dynamic AABB tree broadphase through KairoSpatial
Plane pairing for infinite static planes
Sphere-sphere, sphere-plane, AABB-AABB, and AABB-plane narrowphase
Sequential impulse normal/friction contact solving
Baumgarte-style position correction
Fixed-step PhysicsWorld
Renderer-agnostic debug contacts and AABBs
Deterministic Catch2 tests
```

No visualization or full engine/editor systems belong in this repo yet.

## Modules

```text
PhysicsEngineTypes   step settings and validation
PhysicsMaterial      restitution/friction coefficients and mixing
RigidBody            body records and world inverse inertia helpers
Collider             sphere, plane, AABB colliders and world bounds
Broadphase           KairoSpatial DynamicAABBTree pair generation
Narrowphase          exact V1 contact generation
ContactSolver        sequential impulse and position correction
PhysicsDebug         debug contacts and AABBs
PhysicsWorld         body/collider ownership and fixed-step simulation
```

Use the umbrella module:

```cpp
import Kairo.Foundation.PhysicsEngine;
```

## Example

```cpp
PhysicsWorld world;
world.Settings.VelocityIterations = 10;

RigidBodyDesc sphereDesc;
sphereDesc.Type = BodyType::Dynamic;
sphereDesc.State.Position = Vec3f{ 0.0f, 2.0f, 0.0f };
sphereDesc.Mass = SphereMassProperties(0.5f, 1.0f);

BodyID sphere = world.CreateRigidBody(sphereDesc);
BodyID floor = world.CreateRigidBody(RigidBodyDesc{ .Type = BodyType::Static });

world.AddCollider(sphere, SphereCollider{ 0.5f });
world.AddCollider(floor, PlaneCollider{ Vec3f::Up(), 0.0f });

for (int i = 0; i < 120; ++i)
{
    world.Step(1.0f / 60.0f);
}
```

## Contact Convention

Contact normals point from body A toward body B. The solver applies:

```text
body A: -impulse
body B: +impulse
```

This convention is used consistently by sphere-sphere, sphere-plane, AABB-AABB,
and AABB-plane contacts.

## Current Limits

```text
No OBB, capsule, convex hull, mesh, GJK, or EPA narrowphase yet
No sleeping heuristics beyond the body flag
No joints or constraint graph
No continuous collision detection
No island solver
No terminal or GLFW sandbox in this repo
```
