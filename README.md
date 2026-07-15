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
./build/KairoPhysicsSandbox sphere-collision --steps 180 --every 30
./build/KairoPhysicsSandbox box-stack --steps 240 --every 40
./build/KairoPhysicsSandbox friction-ramp --steps 240 --every 40
./build/KairoPhysicsSandbox restitution-test --steps 240 --every 40
./build/KairoPhysicsSandbox sleeping-test --steps 420 --every 60
./build/KairoPhysicsSandbox stress-100 --steps 180 --every 30 --csv outputs/stress_100.csv
./build/KairoPhysicsSandbox stress-500 --steps 180 --every 30 --csv outputs/stress_500.csv
```

Terminal scenarios:

```text
falling-sphere
sphere-collision
box-stack
friction-ramp
restitution-test
sleeping-test
stress-100
stress-500
```

Terminal output includes frame stats, tracked body transforms/velocities, and
an ASCII side-view. CSV output uses:

```text
frame,time,bodies,contacts,islands,awake,sleeping,step_ms,broadphase_ms,narrowphase_ms,solver_ms
```

Run the GLFW debug sandbox when GLFW/OpenGL are available:

```bash
./build/KairoPhysicsGlfwSandbox
```

GLFW playground controls:

```text
1                 falling sphere demo
2                 sphere collision demo
3                 box stack demo
4                 friction ramp demo
5                 restitution/bounce demo
6                 sleeping stack demo
Delete/Backspace  remove selected object
C                 clear dynamic objects
Tab               select next dynamic object
[                 select previous dynamic object
Left click         select nearest dynamic object
Arrows/WASD        gently push selected object with force
Shift+Arrows/WASD  slowly direct-move selected object
Q/E               apply torque to selected object
L                 toggle physics ray fan
J/K               rotate ray fan around selected object
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
CapsuleCollider with sphere/capsule/plane/AABB/oriented-box contacts
Collider local center and local rotation
BelongsTo / CollidesWith collision filters
Built-in CollisionLayer bits for player/projectile/trigger/sensor/cloth/fluid/particle categories
CollisionResponse rules: Ignore, Trigger, Block
Layer response table and collider-pair response overrides
Runtime collision filter callback
Synchronous Begin/Stay/End contact event callback
Trigger/sensor contacts that report overlap without solver response
Persistent KairoSpatial DynamicAABBTree broadphase
Last-step profiling: total, broadphase, narrowphase, solver timings
Plane pairing for infinite plane colliders
Sphere-sphere, sphere-plane, sphere-box, AABB-AABB, AABB-plane, box-box SAT, and box-plane contacts
Sequential impulse solver with warm-started normal/friction impulses
Separate velocity and position solver iterations
Baumgarte position correction with static-body protection
Fixed timestep accumulator through PhysicsWorld::StepFixed
Frame contact events: Begin, Stay, End
Overlap queries: QueryAABB and QuerySphere
Physics rays: Raycast nearest hit and RaycastAll sorted hit list
Broadphase-backed overlap/raycast candidates plus continuous SweepSphere queries
Dedicated ProjectileSystem: hitscan/ballistic motion, owner ignore, masks, impulses, bounce/pierce/destroy, callbacks
BuoyancySystem: AABB water volumes, displaced-volume buoyancy, drag, Enter/Stay/Exit events
Renderer-agnostic debug contacts, AABBs, and exact collider-shape snapshots
Terminal and GLFW debug sandboxes
Modular terminal sandbox core with scenarios, ASCII rendering, and CSV benchmark output
Catch2 regression tests
```

## Module Map

```text
PhysicsEngineTypes   step settings, fixed-step validation, sleep thresholds
PhysicsMaterial      restitution/friction coefficients and mixing
RigidBody            body records, active flags, sleep/wake helpers
Collider             sphere/capsule/plane/AABB/box colliders, filters, world bounds
Broadphase           persistent KairoSpatial DynamicAABBTree pair generation
Narrowphase          exact V1 contact generation and box SAT
ContactSolver        warm-started sequential impulses and position correction
PhysicsDebug         renderer-agnostic debug contacts and AABBs
PhysicsWorld         ownership, fixed stepping, events, collision response rules, overlap/raycast/sweep queries, sandbox-facing API
ProjectileSystem     continuous gameplay projectile lifecycle and impact behavior
Buoyancy             gameplay water volumes, buoyancy force, drag, volume events
```

Sandbox module map:

```text
SandboxTypes          scenario ids, run settings, tracked body metadata, frame stats
DemoScene             reusable demo/stress scene construction
TerminalRenderer      numeric logs, body rows, ASCII side-view, benchmark CSV
PhysicsDebugRenderer  renderer-agnostic AABB/contact snapshot capture
SandboxApp            CLI parsing and fixed-step terminal run loop
KairoPhysicsSandbox   umbrella module for sandbox tooling
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

## Collision Rules And Callbacks

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

That mask is the coarse broadphase gate. After narrowphase confirms an exact
overlap, the world resolves a `CollisionResponse`:

```text
Ignore   no contact and no event
Trigger  contact/event only, no solver impulse or position correction
Block    contact/event plus solver response
```

Layer response example:

```cpp
world.SetColliderCollisionLayer(playerCollider, CollisionLayer::Player);
world.SetColliderCollisionLayer(pickupCollider, CollisionLayer::Trigger);

world.SetCollisionLayerResponse(
    CollisionLayer::Player,
    CollisionLayer::Trigger,
    CollisionResponse::Trigger);
```

Pair response override example:

```cpp
world.SetCollisionPairResponse(
    projectileCollider,
    ownerCollider,
    CollisionResponse::Ignore);
```

Runtime classification hook:

```cpp
world.SetCollisionFilterCallback(
    [](const Collider& a, const Collider& b)
    {
        return ShouldOverlapOnly(a, b)
            ? CollisionResponse::Trigger
            : CollisionResponse::Block;
    });
```

Contact callback:

```cpp
world.SetContactEventCallback(
    [](const PhysicsContactEvent& event)
    {
        if (event.Type == PhysicsContactEventType::Begin &&
            event.Response == CollisionResponse::Trigger)
        {
            // pickup, sensor enter, laser volume, area trigger, etc.
        }
    });
```

Response resolution order is deterministic:

```text
1. explicit collider-pair response
2. runtime collision filter callback
3. layer response table
4. default mask/trigger behavior
```

The old trigger flag remains as a simple default:

```cpp
world.SetColliderTrigger(sensorCollider, true);
```

Triggers still generate `ContactManifold` records and contact events, but the
solver skips warm starting, impulses, and position correction.

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

The same events can be consumed through `SetContactEventCallback`. The callback
payload contains body ids, collider ids, trigger state, final response, and event
type.

Scene queries intentionally return collider ids, not engine-owned references:

```cpp
std::vector<ColliderID> hits =
    world.QuerySphere(Vec3f{ 0.0f, 1.0f, 0.0f }, 2.0f);
```

Queries currently use active finite collider AABBs. Infinite planes are excluded.

`SweepSphere` is the continuous-query bridge for fast gameplay objects. It
accelerates candidates through the same dynamic AABB tree, includes planes, and
then performs exact shape-distance advancement:

```cpp
const auto hit = world.SweepSphere(
    muzzlePosition,
    launchVelocity * dt,
    projectileRadius,
    CollisionLayer::StaticWorld | CollisionLayer::DynamicWorld,
    ownerCollider);
```

Use `ProjectileSystem` when a projectile needs lifecycle management rather than
hand-writing a sweep every frame:

```cpp
ProjectileSystem projectiles;
ProjectileDesc round;
round.Mode = ProjectileMode::Ballistic;
round.Position = muzzlePosition;
round.Velocity = launchVelocity;
round.IgnoredOwnerCollider = ownerCollider;
projectiles.Spawn(round);

projectiles.Step(world, dt); // before or after world.Step(dt), by game policy
world.Step(dt);
```

Gameplay water is intentionally separate from particle fluids. Configure its
physical displaced volume explicitly, then apply it before the rigid-body step:

```cpp
BuoyancySystem water;
water.AddWaterVolume({ .Bounds = waterBounds, .Density = 1.0f });
water.RegisterBody(boatBody, { .DisplacedVolume = 2.4f });

water.Step(world, dt);
world.Step(dt);
```

## Current Limits

Still deferred:

```text
Convex hulls, mesh colliders, GJK, EPA
Continuous dynamic rigid-body CCD beyond query/projectile sweeps
Island solver and parallel island dispatch
Joints and articulated constraints
Persistent multi-point contact manifolds
Serialization/replay file format
Full editor/ImGui tooling
```

Those belong to later engine phases, not `KairoPhysicsMath`.

## Physics Roadmap

The rigid body engine now has the filtering, response, query, and callback
surface needed before larger physics families are added. The next families
should be separate modules that reuse the same world/query/event conventions:

```text
KairoParticles      particle emitters, forces, constraints, collision events
KairoCloth          mass-spring or XPBD cloth, cloth collision, tearing later
KairoFluids         SPH or grid fluids, rigid-body coupling, surface interaction
KairoDestruction    fracture pieces, debris, sleeping/island integration
KairoVehicles       raycast wheels, suspension, tire friction curves
```

The near-term rule is not to fake those systems inside rigid bodies. Each one
should land with tests, documentation, sandbox demos, and clear integration
points.
