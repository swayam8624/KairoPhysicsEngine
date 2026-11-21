# KairoPhysicsEngine — guided simulation source order

Build the mental model of **one deterministic fixed-timestep physics update**. A renderer shows the result but does not calculate it. KairoPlayer's [RuntimePhysicsBridge](https://github.com/swayam8624/KairoGameEngine/blob/main/Runtime/KairoPlayer/RuntimePhysicsBridge.cppm) creates the world from authored scene descriptors, advances it and copies/interpolates poses back into the runtime. PhysicsEngine itself depends on [KairoMath](https://github.com/swayam8624/KairoMath), [KairoGeometry](https://github.com/swayam8624/KairoGeometry), [KairoSpatial](https://github.com/swayam8624/KairoSpatial), and [KairoPhysicsMath](https://github.com/swayam8624/KairoPhysicsMath).

```text
Body + Collider states  →  integrate forces / fixed-step validation
                                    |
                              Broadphase (AABB pairs)
                                    |
                              Narrowphase (true contacts)
                                    |
                      ContactSolver + Joint constraints
                                    |
                             SolverIsland partitioning
                                    |
                     positions, velocities, sleep / events
                                    |
                 replay / debug extraction / Player interpolation
```

## Recommended reading order

1. [`KairoPhysicsEngine.cppm`](KairoPhysicsEngine.cppm) and [`PhysicsEngineTypes.cppm`](PhysicsEngineTypes.cppm): umbrella module, public types, IDs, time settings and validation. Each `export import` names another C++23 module.
2. [`RigidBody.cppm`](RigidBody.cppm) and [`PhysicsMaterial.cppm`](PhysicsMaterial.cppm): mass, inertia, positions, velocities, forces, friction and restitution; distinguish dynamic/kinematic/static motion.
3. [`Collider.cppm`](Collider.cppm): shape geometry, transforms, collision masks and trigger semantics. A body may be represented by one or more colliders.
4. [`Broadphase.cppm`](Broadphase.cppm): find **candidate** pairs by bounding volumes. False positives are okay; missing true contacts is not. Compare to all-pairs results.
5. [`ConvexCollision.cppm`](ConvexCollision.cppm) and [`Narrowphase.cppm`](Narrowphase.cppm): precise shape intersection/contact geometry from broadphase candidates, penetration depth, normals and contact manifolds.
6. [`ContactSolver.cppm`](ContactSolver.cppm): sequential impulses, contact correction, friction/restitution and warm starting. Read the contact-normal sign convention before modifying the solver.
7. [`Joint.cppm`](Joint.cppm) and [`SolverIsland.cppm`](SolverIsland.cppm): constraints across connected rigid bodies, independent islands, solving/sleeping choices.
8. [`PhysicsWorld.cppm`](PhysicsWorld.cppm): **the integration point** for the entire pipeline. Track a single fixed step from input validation through forces, candidate pairs, contact manifolds, constraint solving, poses, callbacks and queries. Keep mutation during callbacks separate from currently executing step.
9. [`CharacterController.cppm`](CharacterController.cppm), [`ProjectileSystem.cppm`](ProjectileSystem.cppm), [`VehicleSystem.cppm`](VehicleSystem.cppm), and [`Buoyancy.cppm`](Buoyancy.cppm): gameplay adapters, not replacements for robust core collision/solver contracts.
10. [`PhysicsReplay.cppm`](PhysicsReplay.cppm), [`PhysicsSerialization.cppm`](PhysicsSerialization.cppm) and [`PhysicsDebug.cppm`](PhysicsDebug.cppm): repeatable steps, saved state, and debug drawing. Debug visualization is a consumer of physics output, not the authoritative solver.
11. [`sandbox/KairoPhysicsSandbox.cppm`](sandbox/KairoPhysicsSandbox.cppm), [`sandbox/KairoPhysicsGlfwSandbox.cpp`](sandbox/KairoPhysicsGlfwSandbox.cpp), and [`sandbox/examples/main.cpp`](sandbox/examples/main.cpp): demonstrate terminal/GLFW examples; each is an experiment, not automatically proof of stable stacking.
12. [`tests/PhysicsEngineTests.cpp`](tests/PhysicsEngineTests.cpp), [`tests/PhysicsStressTests.cpp`](tests/PhysicsStressTests.cpp), [`tests/CharacterControllerTests.cpp`](tests/CharacterControllerTests.cpp), and [`tests/VehicleSystemTests.cpp`](tests/VehicleSystemTests.cpp): compare simulation behavior, invariants and edge cases.
13. [`CMakeLists.txt`](CMakeLists.txt), [`STATUS.md`](STATUS.md): build and native acceptance; the earlier 95% claim is retracted pending exact-head integrated tests.

## APIs and components

- **`RigidBody`** owns physical state, **`Collider`** owns contact geometry and filters; don't mix visual mesh triangles with collision shapes by accident.
- **`Broadphase`** returns potential overlaps; **`Narrowphase`** returns physical contacts; **`ContactSolver`** modifies velocities/impulses; **`PhysicsWorld`** orchestrates them.
- **`SolverIsland`** partitions coupled bodies/constraints; changes to island iteration can change determinism or parallel behavior.
- **`PhysicsReplay`** supports reproducible state/step observation; identical input and fixed timestep are prerequisites for meaningful comparison.
- **`KairoPlayer.RuntimePhysicsBridge`** is a **separate repository module** that adapts authored scenes to PhysicsEngine and interpolates poses for the renderer.

A concrete exercise: simulate a falling sphere onto a static plane, recording candidate pairs, contact normals, pre/post-impulse velocity, penetration correction, and sleeping threshold each step. Re-run with a small different time step and compare stability; verify physics independently of the screen output.

The main [MIT license](LICENSE) applies to original KairoPhysicsEngine code; dependencies and external models retain their terms. This guide is educational documentation, not a completed full-system or all-host verification certificate.
