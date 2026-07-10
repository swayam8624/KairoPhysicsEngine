# KairoPhysicsEngine

`KairoPhysicsEngine` is the rigid-body simulation layer in the Kairo foundation
stack.

```text
KairoMath -> KairoPhysicsMath -> KairoGeometry -> KairoSpatial -> KairoPhysicsEngine
```

It owns body/collider storage, broadphase, narrowphase, contact solving, fixed
stepping, debug extraction, scene queries, contact events, and sandbox tooling.
It is still not the final game runtime, editor, character controller package, or
joint/constraint graph.

## Quickstart

```bash
cd /Users/swayamsingal/Desktop/Programming/Kairo/Foundation/KairoPhysicsEngine
cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++
cmake --build build
ctest --test-dir build --output-on-failure
```

Run the terminal sandbox:

```bash
./build/KairoPhysicsSandbox falling-sphere
./build/KairoPhysicsSandbox trigger
./build/KairoPhysicsSandbox rotated-box
```

Run the GLFW debug sandbox when GLFW/OpenGL are available:

```bash
./build/KairoPhysicsGlfwSandbox
```

GLFW playground controls:

```text
1                 add sphere
2                 add box
3                 add a small stack
Delete/Backspace  remove selected object
C                 clear dynamic objects
Tab               select next dynamic object
[                 select previous dynamic object
Left click         select nearest dynamic object
Arrows/WASD        gently push selected object with force
Shift+Arrows/WASD  slowly direct-move selected object
Q/E               apply torque to selected object
Space             pause/resume
N                 single fixed step while paused
R                 reset playground
G                 toggle gravity
T                 toggle AABB debug overlay
Esc               close
```

## Dependency Resolution

Dependencies resolve local Kairo repos first:

```text
KAIRO_PHYSICS_MATH_SOURCE_DIR -> ../KairoPhysicsMath -> GitHub
KAIRO_SPATIAL_SOURCE_DIR      -> ../Spatial          -> GitHub
```

`KairoSpatial` itself resolves `KairoGeometry`, and the lower repos resolve
`KairoMath`. No outside math library is used.

## Implemented Surface

Current engine surface:

```text
Stable vector-index body/collider ids with inactive deletion-safe records
Dynamic, static, and kinematic body behavior
Per-body gravity scale, damping, max velocity clamps, sleeping, and wake hooks
Sphere, plane, AABB, and oriented BoxCollider shapes
Collider local center and local rotation
BelongsTo / CollidesWith collision filters
Trigger/sensor contacts that report overlap without solver response
Persistent KairoSpatial DynamicAABBTree broadphase
Plane pairing for infinite plane colliders
Sphere-sphere, sphere-plane, sphere-box, AABB-AABB, AABB-plane, box-box SAT, and box-plane contacts
Sequential impulse solver with warm-started normal/friction impulses
Separate velocity and position solver iterations
Baumgarte position correction with static-body protection
Fixed timestep accumulator through PhysicsWorld::StepFixed
Frame contact events: Begin, Stay, End
Overlap queries: QueryAABB and QuerySphere
Renderer-agnostic debug contacts and AABBs
Terminal and GLFW debug sandboxes
Catch2 regression tests
```

## Module Map

```text
PhysicsEngineTypes   step settings, fixed-step validation, sleep thresholds
PhysicsMaterial      restitution/friction coefficients and mixing
RigidBody            body records, active flags, sleep/wake helpers
Collider             sphere/plane/AABB/box colliders, filters, world bounds
Broadphase           persistent KairoSpatial DynamicAABBTree pair generation
Narrowphase          exact V1 contact generation and box SAT
ContactSolver        warm-started sequential impulses and position correction
PhysicsDebug         renderer-agnostic debug contacts and AABBs
PhysicsWorld         ownership, fixed stepping, events, queries, sandbox-facing API
```

Use the umbrella module:

```cpp
import Kairo.Foundation.PhysicsEngine;
```

## Minimal Example

```cpp
PhysicsWorld world;
world.Settings.VelocityIterations = 16;
world.Settings.PositionIterations = 6;

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

## Filters And Triggers

Collision filtering is symmetric:

```cpp
world.SetCollisionFilter(playerCollider, 0b0001u, 0b0110u);
world.SetCollisionFilter(enemyCollider,  0b0010u, 0b0001u);
```

A pair is considered only when both directions agree:

```text
A.CollidesWith contains B.BelongsTo
B.CollidesWith contains A.BelongsTo
```

Triggers still generate `ContactManifold` records and contact events, but the
solver skips warm starting, impulses, and position correction:

```cpp
world.SetColliderTrigger(sensorCollider, true);
```

## Contact Convention

Contact normals point from body A toward body B. The solver applies:

```text
body A: -impulse
body B: +impulse
```

Swapped plane/sphere and plane/box paths are tested so this convention remains
stable across pair ordering.

## Events And Queries

`PhysicsWorld::ContactEvents()` returns deterministic per-step events:

```text
Begin
Stay
End
```

Scene queries intentionally return collider ids, not engine-owned references:

```cpp
std::vector<ColliderID> hits =
    world.QuerySphere(Vec3f{ 0.0f, 1.0f, 0.0f }, 2.0f);
```

Queries currently use active finite collider AABBs. Infinite planes are excluded.

## Current Limits

Still deferred:

```text
Capsule collider in engine narrowphase
Convex hulls, mesh colliders, GJK, EPA
Continuous collision detection
Island solver and parallel island dispatch
Joints and articulated constraints
Persistent multi-point contact manifolds
Serialization/replay file format
Full editor/ImGui tooling
```

Those belong to later engine phases, not `KairoPhysicsMath`.
