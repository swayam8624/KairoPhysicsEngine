# Teaching source map — KairoPhysicsEngine

> A guided source-reading itinerary. It is not a test result or claim that every file is documented yet.

## How to use this guide

Start with IDs and colliders; follow a time step through broadphase, narrowphase and constraint solving, then inspect high-level gameplay helpers and replay.

Read each phase in order. Every link goes to a source file or directory in this repository. For a directory, inspect its public interface and then its implementation files. After each phase, identify the caller, the state it owns, the inputs it validates, and its observable outputs.

## 1. Basic types

1. [`README.md`](../README.md)
2. [`PhysicsEngineTypes.cppm`](../PhysicsEngineTypes.cppm)
3. [`RigidBody.cppm`](../RigidBody.cppm)
4. [`Collider.cppm`](../Collider.cppm)
5. [`PhysicsMaterial.cppm`](../PhysicsMaterial.cppm)

## 2. One simulation step

1. [`Broadphase.cppm`](../Broadphase.cppm)
2. [`Narrowphase.cppm`](../Narrowphase.cppm)
3. [`ConvexCollision.cppm`](../ConvexCollision.cppm)
4. [`ContactSolver.cppm`](../ContactSolver.cppm)
5. [`SolverIsland.cppm`](../SolverIsland.cppm)
6. [`Joint.cppm`](../Joint.cppm)
7. [`PhysicsWorld.cppm`](../PhysicsWorld.cppm)

## 3. Gameplay and observability

1. [`CharacterController.cppm`](../CharacterController.cppm)
2. [`VehicleSystem.cppm`](../VehicleSystem.cppm)
3. [`ProjectileSystem.cppm`](../ProjectileSystem.cppm)
4. [`Buoyancy.cppm`](../Buoyancy.cppm)
5. [`PhysicsDebug.cppm`](../PhysicsDebug.cppm)
6. [`PhysicsReplay.cppm`](../PhysicsReplay.cppm)
7. [`PhysicsSerialization.cppm`](../PhysicsSerialization.cppm)

## 4. Verification

1. [`tests/PhysicsEngineTests.cpp`](../tests/PhysicsEngineTests.cpp)
2. [`tests/PhysicsStressTests.cpp`](../tests/PhysicsStressTests.cpp)
3. [`tests/CharacterControllerTests.cpp`](../tests/CharacterControllerTests.cpp)
4. [`tests/VehicleSystemTests.cpp`](../tests/VehicleSystemTests.cpp)
5. [`sandbox`](../sandbox)

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
