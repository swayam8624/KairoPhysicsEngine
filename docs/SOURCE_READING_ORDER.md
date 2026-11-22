# Teaching source map — KairoPhysicsEngine

> A guided source-reading itinerary. It is not a test result or claim that every file is documented yet.

## How to use this guide

For a narrative explanation of the complete simulation and its mathematical
and spatial dependencies, read the [collision pipeline walkthrough](COLLISION_PIPELINE_WALKTHROUGH.md).
It cross-links KairoMath, KairoPhysicsMath, KairoGeometry and KairoSpatial.

**Solve the mathematics while reading:** [Contact impulses, friction, warm starts and solver islands](CONTACT_SOLVER_FROM_EQUATIONS_TO_CODE.md) includes a worked two-mass collision and regression exercises.

Start with IDs and colliders; follow a time step through broadphase, narrowphase and constraint solving, then inspect high-level gameplay helpers and replay.

Read each phase in order. Every link goes to a source file or directory in this repository. For a directory, inspect its public interface and then its implementation files. After each phase, identify the caller, the state it owns, the inputs it validates, and its observable outputs.

**Extended source exercise:** [Trace BodyID and ColliderID lifetime, one full Step, trigger Begin/Stay/End, warm-started contacts, snapshot restore and continuous-impact limits](BODY_CONTACT_EVENT_AND_CCD_LIFECYCLE_LAB.md). Distinguish the PhysicsWorld fixed-step backlog policy from the Player adapter.

## 1. Build, entry point and state identity

1. [`CMakeLists.txt`](../CMakeLists.txt): configure modules, dependencies and tests.
2. [`KairoPhysicsEngine.cppm`](../KairoPhysicsEngine.cppm): umbrella interface; exports are not a call sequence.
3. [`PhysicsEngineTypes.cppm`](../PhysicsEngineTypes.cppm): entity IDs, enums and shared policies.
4. [`RigidBody.cppm`](../RigidBody.cppm): body motion state, mass/inertia and active/inactive behavior.
5. [`Collider.cppm`](../Collider.cppm): shapes, local offsets, material/filter and ownership.
6. [`PhysicsMaterial.cppm`](../PhysicsMaterial.cppm): restitution and friction input contracts.

**Trace:** an identity for a body need not identify one collider: one body
can own multiple shapes. Check deletion/inactive records before indexing
and distinguish body-local from world-space quantities.

## 2. Follow one fixed simulation step

7. [`PhysicsWorld.cppm`](../PhysicsWorld.cppm): start with `PhysicsWorld::Step` and trace call order.
8. [`Broadphase.cppm`](../Broadphase.cppm): finite-collider AABB proxies in KairoSpatial, plus infinite-collider path.
9. [`Narrowphase.cppm`](../Narrowphase.cppm): test candidate pairs and generate contact data.
10. [`ConvexCollision.cppm`](../ConvexCollision.cppm): convex query geometry and numerical edge cases.
11. [`SolverIsland.cppm`](../SolverIsland.cppm): connected constraint groups and solver partitioning.
12. [`ContactSolver.cppm`](../ContactSolver.cppm): normal/friction responses, bias and cached impulses.
13. [`Joint.cppm`](../Joint.cppm): joint constraints that share rigid-body solver state.
14. Return to [`PhysicsWorld.cppm`](../PhysicsWorld.cppm): examine contact events, state update and profiling.

**Trace:** collider changes -> broadphase candidate pair -> geometric contact
-> constraint row/island -> iterative impulse -> updated velocities and
contact Begin/Stay/End. Identify when sensors/triggers skip physical response.
AABB overlap alone does not prove collision; continuous sweep queries
should not be confused with discrete contact resolution.

## 3. Gameplay systems and observability

15. [`CharacterController.cppm`](../CharacterController.cppm): movement against scene geometry and collision responses.
16. [`VehicleSystem.cppm`](../VehicleSystem.cppm): vehicle controls and forces/contacts.
17. [`ProjectileSystem.cppm`](../ProjectileSystem.cppm): projectiles and continuous query use.
18. [`Buoyancy.cppm`](../Buoyancy.cppm): displaced-fluid-related forces and assumptions.
19. [`PhysicsDebug.cppm`](../PhysicsDebug.cppm): read-only geometric diagnostics.
20. [`PhysicsReplay.cppm`](../PhysicsReplay.cppm): capture/replay state needed for reproducibility.
21. [`PhysicsSerialization.cppm`](../PhysicsSerialization.cppm): persistence/version and schema contracts.

**Trace:** distinguish a renderer-facing debug view of state from the
authoritative simulation state; a saved body pose alone may be insufficient
to reproduce solver caches and contact event continuity.

## 4. Tests and runnable demonstration

22. [`tests/PhysicsEngineTests.cpp`](../tests/PhysicsEngineTests.cpp): basic state, collision and world behavior.
23. [`tests/PhysicsStressTests.cpp`](../tests/PhysicsStressTests.cpp): workload and numerical stress cases.
24. [`tests/CharacterControllerTests.cpp`](../tests/CharacterControllerTests.cpp): controller contracts.
25. [`tests/VehicleSystemTests.cpp`](../tests/VehicleSystemTests.cpp): vehicle behavior.
26. [`sandbox/examples/main.cpp`](../sandbox/examples/main.cpp): terminal demo invocation.
27. [`sandbox/DemoScene.cppm`](../sandbox/DemoScene.cppm): actual scenario setup.
28. [`sandbox/KairoPhysicsSandbox.cppm`](../sandbox/KairoPhysicsSandbox.cppm): sandbox API and workflow.
29. [`sandbox/KairoPhysicsGlfwSandbox.cpp`](../sandbox/KairoPhysicsGlfwSandbox.cpp): optional windowed runner.
30. [`sandbox/PhysicsDebugRenderer.cppm`](../sandbox/PhysicsDebugRenderer.cppm): diagnostics rendering.

**Check:** record which binaries and tests actually ran on the host,
the compiler configuration, scenario, timestep and output. A readable
demo is not evidence of a numerically validated solver.

## End-to-end interaction map

`PhysicsWorld` maintains bodies and colliders, advances fixed time, updates spatial broadphase proxies, creates candidate pairs, computes actual contacts in narrowphase, solves islands/constraints, emits events, and exports transforms/debug data to EngineCore and Renderer adapters.

## APIs and code-reading questions

For each function, distinguish **contract** (preconditions, postconditions, failure modes), **implementation** (state/algorithms), **collaborators** (imports, calls and consumers), **lifetime/ownership**, and **verification** (test or evidence that checks it). For C++ modules, `export module` names the public compiled interface and `import` expresses module dependencies; an implementation unit's `module ...;` contributes definitions without exporting them.

## Boundaries and honesty

Broadphase candidates are not guaranteed contacts. Solver stability, determinism and native visualization require separate tests; cloth/fluids/destruction are post-v1.

## Documentation rules

- Teaching comments describe *why*, invariants, edge cases, API ownership and cross-file effects; avoid narrating obvious assignments.
- Preserve behavior during documentation-only edits; keep correctness changes in separate reviewed commits.
- Link function names and tests where possible; mark claims as source-present, CI-observed, host-verified or research-evidenced.
- License grants apply only to material for which the repository owner holds rights; third-party source, models, media and datasets keep their own notices.


## Complete source-file inventory

After the curated teaching path above, use this ordered checklist to reach every first-party source, script, shader and test file indexed in this branch. Group order follows concepts before operations, tooling and evidence; follow module imports when a different within-group order is more helpful. Vendored/dependency code, binaries, data and generated artifacts are excluded, not relicensed.

### Entry points and build (2)

1. [`CMakeLists.txt`](../CMakeLists.txt)
2. [`sandbox/examples/main.cpp`](../sandbox/examples/main.cpp)

### Interfaces and modules (25)

1. [`Broadphase.cppm`](../Broadphase.cppm)
2. [`Buoyancy.cppm`](../Buoyancy.cppm)
3. [`CharacterController.cppm`](../CharacterController.cppm)
4. [`Collider.cppm`](../Collider.cppm)
5. [`ContactSolver.cppm`](../ContactSolver.cppm)
6. [`ConvexCollision.cppm`](../ConvexCollision.cppm)
7. [`Joint.cppm`](../Joint.cppm)
8. [`KairoPhysicsEngine.cppm`](../KairoPhysicsEngine.cppm)
9. [`Narrowphase.cppm`](../Narrowphase.cppm)
10. [`PhysicsDebug.cppm`](../PhysicsDebug.cppm)
11. [`PhysicsEngineTypes.cppm`](../PhysicsEngineTypes.cppm)
12. [`PhysicsMaterial.cppm`](../PhysicsMaterial.cppm)
13. [`PhysicsReplay.cppm`](../PhysicsReplay.cppm)
14. [`PhysicsSerialization.cppm`](../PhysicsSerialization.cppm)
15. [`PhysicsWorld.cppm`](../PhysicsWorld.cppm)
16. [`ProjectileSystem.cppm`](../ProjectileSystem.cppm)
17. [`RigidBody.cppm`](../RigidBody.cppm)
18. [`sandbox/DemoScene.cppm`](../sandbox/DemoScene.cppm)
19. [`sandbox/KairoPhysicsSandbox.cppm`](../sandbox/KairoPhysicsSandbox.cppm)
20. [`sandbox/PhysicsDebugRenderer.cppm`](../sandbox/PhysicsDebugRenderer.cppm)
21. [`sandbox/SandboxApp.cppm`](../sandbox/SandboxApp.cppm)
22. [`sandbox/SandboxTypes.cppm`](../sandbox/SandboxTypes.cppm)
23. [`sandbox/TerminalRenderer.cppm`](../sandbox/TerminalRenderer.cppm)
24. [`SolverIsland.cppm`](../SolverIsland.cppm)
25. [`VehicleSystem.cppm`](../VehicleSystem.cppm)

### Implementation and other source (1)

1. [`sandbox/KairoPhysicsGlfwSandbox.cpp`](../sandbox/KairoPhysicsGlfwSandbox.cpp)

### Verification and tests (4)

1. [`tests/CharacterControllerTests.cpp`](../tests/CharacterControllerTests.cpp)
2. [`tests/PhysicsEngineTests.cpp`](../tests/PhysicsEngineTests.cpp)
3. [`tests/PhysicsStressTests.cpp`](../tests/PhysicsStressTests.cpp)
4. [`tests/VehicleSystemTests.cpp`](../tests/VehicleSystemTests.cpp)

Index snapshot: 32 first-party source/build/test files among 53 tracked files. The curated list remains the conceptual starting point. This inventory alone does not mean those functions have already been annotated or tested.
