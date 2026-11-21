# KairoPhysicsEngine — guided source reading order

This guide follows the **simulation data flow**, not alphabetical source order. It links to implemented C++23 modules and connects the standalone physics package to the [KairoGameEngine player](https://github.com/swayam8624/KairoGameEngine/blob/main/Runtime/KairoPlayer/RuntimePhysicsBridge.cppm). Read [README](../README.md) and [CMakeLists](../CMakeLists.txt) first for supported backends, build flags and test targets.

## 1. Understand the physical state

1. [PhysicsEngineTypes.cppm](../PhysicsEngineTypes.cppm) — identifiers, simulation settings, and cross-module contracts.
2. [RigidBody.cppm](../RigidBody.cppm) — mass, inertia, velocities and motion modes; learn the difference between dynamic, static and kinematic bodies.
3. [PhysicsMaterial.cppm](../PhysicsMaterial.cppm) — friction/restitution coefficients and combining rules.
4. [Collider.cppm](../Collider.cppm) — shapes, body association, broadphase bounds and collision filters.
5. [KairoPhysicsEngine.cppm](../KairoPhysicsEngine.cppm) — umbrella import/re-export surface. Understand which type belongs to which module before reading World.

## 2. Learn collision detection before response

6. [Broadphase.cppm](../Broadphase.cppm) — uses [KairoSpatial](https://github.com/swayam8624/KairoSpatial) to query potentially overlapping collider pairs; AABB overlap is **not** itself physical contact.
7. [ConvexCollision.cppm](../ConvexCollision.cppm) — convex support-mapping and geometric contact helpers.
8. [Narrowphase.cppm](../Narrowphase.cppm) — filters/refines candidates, builds contact manifolds and reports contact geometry.
9. [ContactSolver.cppm](../ContactSolver.cppm) — applies impulses/constraints to bodies using contact manifolds and material properties. Find warm-starting and the treatment of trigger contacts.
10. [SolverIsland.cppm](../SolverIsland.cppm), [Joint.cppm](../Joint.cppm) — groups interacting constraints and applies joint restrictions.

## 3. Put the simulation together

11. [PhysicsWorld.cppm](../PhysicsWorld.cppm) — top-level owner of body/collider state and the step pipeline; trace integration → broadphase → narrowphase → island/contact/joint solving → events.
12. [CharacterController.cppm](../CharacterController.cppm), [VehicleSystem.cppm](../VehicleSystem.cppm), [ProjectileSystem.cppm](../ProjectileSystem.cppm) and [Buoyancy.cppm](../Buoyancy.cppm) — consumers of world stepping and collision queries.
13. [PhysicsDebug.cppm](../PhysicsDebug.cppm) — transforms and contact state exported for visualization, without making the solver dependent on an editor.

## 4. Durability and experimental validation

14. [PhysicsSerialization.cppm](../PhysicsSerialization.cppm) — explicit snapshot formats, versioning, binary bounds and durable state.
15. [PhysicsReplay.cppm](../PhysicsReplay.cppm) — initial snapshot + time-stepped commands + expected state hashes. Determinism must be tested under controlled compiler/platform/configuration conditions.
16. [docs/PHYSICS_V1_STABILITY.md](PHYSICS_V1_STABILITY.md) and [docs/SERIALIZATION_REPLAY.md](SERIALIZATION_REPLAY.md) — intended invariants and validation/replay expectations.
17. [tests](../tests/) — inspect tests targeting collision combinations, solver stability, replay and serialization.
18. [sandbox](../sandbox/) — run visualization/terminal scenarios after the unit tests; see [README](../README.md) for command lines.
19. [research/adaptive_collision_pipeline.yaml](../research/adaptive_collision_pipeline.yaml) — research planning/evaluation is separate from the implemented core.

## Cross-file call traces

**Collision:** `PhysicsWorld` updates AABB proxies in `BroadphaseWorld` → candidate collider ID pairs → `Narrowphase` computes actual manifolds → `ContactSolver` updates body velocities/impulses → world integration and events. This separates *collision possibility*, *contact geometry*, and *physical response*.

**Runtime gameplay:** KairoPlayer's `RuntimePhysicsBridge` maps EngineCore entities to the physics world's session-local IDs, advances fixed steps and translates contact events back into entity-facing events. Never serialize process-local physics IDs as stable editor identity.

**Reproducibility:** `PhysicsSerialization` captures initial state; `PhysicsReplay` applies recorded commands over fixed timesteps and compares state hashes. A replay passing once does not certify deterministic behaviour across arbitrary compilers and hardware.

## Exercises

1. Track two overlapping AABBs that do not have a true shape contact. Locate where the false positive is rejected.
2. Find how sleeping bodies, trigger colliders and contact restitution change the step path.
3. Draw the difference between broadphase pairs and manifolds.
4. Trace a contact event from `PhysicsWorld` into KairoPlayer.
5. Record and replay a short simulation, and investigate which choices affect state hashes.

Original code is intended for use, learning and further development under [LICENSE](../LICENSE). External dependencies remain subject to their own licenses.
