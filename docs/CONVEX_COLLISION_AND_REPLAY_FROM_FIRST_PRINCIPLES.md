# From support points to reproducible contacts — a KairoPhysicsEngine C++23 laboratory

This chapter continues [the physics reading order](SOURCE_READING_ORDER.md), [the full collision pipeline](COLLISION_PIPELINE_WALKTHROUGH.md), [the equations-to-contact-solver guide](CONTACT_SOLVER_FROM_EQUATIONS_TO_CODE.md), and [body/contact lifecycle](BODY_CONTACT_EVENT_AND_CCD_LIFECYCLE_LAB.md). It specifically traces two previously under-explained first-party modules:

- [`ConvexCollision.cppm`](../ConvexCollision.cppm): shape support maps, Minkowski difference, GJK simplex geometry, EPA penetration, convex hull ray/distance queries
- [`PhysicsReplay.cppm`](../PhysicsReplay.cppm): snapshot + external command log, fixed-step execution, hash divergence, versioned binary file I/O

These are **implementation reading and numerical reasoning exercises**, not newly executed tests. The existing [Catch2 acceptance source](../tests/PhysicsEngineTests.cpp) supplies concrete fixtures. Native build/test status should be established on the exact Git commit and host before a performance or reliability claim is made.

## 1. One physics frame has distinct layers

The call path in [`PhysicsWorld.cppm`](../PhysicsWorld.cppm) is:

```text
PhysicsWorld::Step(dt)
   -> IntegrateForces, IntegrateKinematicVelocities
   -> BroadphaseWorld::Sync          [refresh finite AABB tree + infinite list]
   -> BroadphaseWorld::ComputePairs  [CONSERVATIVE candidate ColliderID pairs]
   -> Narrowphase::ComputeContacts   [actual geometrical manifolds]
       -> CollidePair
           -> CollideConvex          [finite convex support-map case]
   -> ApplyCollisionResponses       [Ignore / Trigger / Block]
   -> UpdateContactEvents           [Begin / Stay / End]
   -> BuildSolverIslands            [dynamic-body connected constraints]
   -> RestoreContactCache, WarmStartContacts
   -> SolveSolverIslands            [normal/friction velocity impulses]
   -> StoreContactCache, CorrectSolverIslandPositions
   -> IntegrateDynamicVelocitiesWithCCD
   -> UpdateSleeping, ClearForceAccumulators
   -> DispatchContactEvents         [host code invoked after solver state]
```

Three results must not be confused: a broadphase pair means the shapes **might** touch; GJK says whether their finite convex Minkowski support structure suggests an overlap under bounded numerical tolerances; a physics contact needs a **normal, depth, witness position, policy and solver response**. Trigger manifolds report events without applying a response impulse.

`BodyID` identifies a motion state. `ColliderID` identifies one shape and local offset owned by that body. One body can have multiple colliders. `SpatialIndex` in KairoSpatial's dynamic AABB tree is a temporary *cache handle*, not either kind of physics identity.

## 2. A support function turns convex geometry into a directional query

For a finite convex set `S`, its support point in direction `d` is

```text
Support(S, d) = argmax_{x in S} dot(x, d)
```

[`ConvexShapeSupport`](../ConvexCollision.cppm) handles several types:

| Collider | Implementation idea | Geometric assumption |
| --- | --- | --- |
| Sphere | center + normalize(d) * radius | radius is positive and the body pose is valid |
| Capsule | farther transformed segment endpoint + normalize(d) * radius | capsule axis is a line segment, radial shell is spherical |
| AABB | per-axis min/max corner chosen by direction signs | world-axis-aligned extent |
| Oriented box | world box frame axes times signed half-extents | valid orthonormal orientation |
| Convex hull | rotate every authoring vertex and retain greatest directional dot | hull was validated as convex/closed and has vertices |

The function rejects infinite planes because there is no finite maximum dot value for arbitrary directions. Those shapes use the separate plane branch in [`Narrowphase.cppm`](../Narrowphase.cppm). Zero-length directions use a normalized fallback to avoid a NaN from vector normalization.

For two shapes `A` and `B`, the Minkowski difference is the set of all point differences `A-B = { a-b }`. We can query it without explicitly generating every pair of vertices:

```text
Support(A-B, d) = Support(A,d) - Support(B,-d)
```

[`ConvexMinkowskiSupport`](../ConvexCollision.cppm) returns all three pieces: the difference point and original world-space witnesses `PointA`, `PointB`. Keeping witnesses is crucial. At the end, EPA can estimate an actual contact location; without witnesses, a Minkowski-space penetration direction alone is not enough to recover the individual surface positions.

## 3. GJK is a geometric search, not a contact solver

The Gilbert–Johnson–Keerthi idea is to ask whether **the origin lies inside A-B**. If it does, `A` and `B` intersect. The code searches a simplex with at most four vertices in 3D, always inserting the newest support point at index zero.

[`UpdateSimplex`](../ConvexCollision.cppm) delegates to the appropriate origin-region test:

- **One vertex:** search from that Minkowski point toward zero.
- **Two vertices (line):** if zero lies in the edge's useful Voronoi direction, search in the perpendicular triple-cross direction; otherwise discard the old endpoint.
- **Three vertices (triangle):** test face, edges and outside regions. Reduce the simplex when zero is closest to an edge, or choose a face normal oriented toward zero.
- **Four vertices (tetrahedron):** test faces opposite the newest point. If zero lies outside a face, keep that triangle; if not, the tetrahedron encloses zero.

The code's triple-cross direction is `(edge × towardOrigin) × edge`. It is perpendicular to the edge and points toward the origin when the geometry is nondegenerate. Small vectors fall back to a chosen perpendicular axis. Triangle and tetrahedron vertex swaps matter because cross products change sign when winding reverses.

The GJK loop terminates on an enclosing tetrahedron, or without a collision report when a new support fails to advance in the search direction, a duplicate support is found, or the numeric/iteration cap prevents resolution. Importantly, **`std::nullopt` can be separation *or* degeneracy/nonconvergence**; treating it as a universal mathematical proof of no intersection would overstate this implementation.

## 4. EPA turns an overlap simplex into penetration depth

The Expanding Polytope Algorithm begins with the tetrahedron found by GJK. Each triangular face stores an outward normal and a nonnegative distance to the origin.

On each bounded EPA iteration, the code:

1. Finds the face closest to the origin.
2. Requests a new Minkowski support point in that face normal direction.
3. Compares the new projection with the face's current distance. If improvement is sufficiently small, convergence is declared.
4. Otherwise removes faces visible from the new support point, cancels internal reverse-oriented edges and constructs new faces connecting the horizon to that new vertex.

The edge cancellation in [`AddBoundaryEdge`](../ConvexCollision.cppm) is a small but important topological algorithm: an edge shared in opposite directions belongs to an internal border and should disappear; edges left in the list delineate the silhouette of removed faces.

At convergence, barycentric coordinates on the selected face interpolate the *original support witnesses*:

```text
wA = alpha*A0 + beta*A1 + gamma*A2
wB = alpha*B0 + beta*B1 + gamma*B2
contactPosition = (wA+wB)/2
penetrationDepth = distance(origin, closest EPA face)
```

The normal is oriented to agree with body A → body B center ordering when needed. This orientation feeds the signed impulses in [`ContactSolver.cppm`](../ContactSolver.cppm). If an EPA face degenerates, a duplicate support occurs, the horizon is missing, or the iteration cap is reached, the implementation can reject the result rather than fabricating a contact.

An exact geometric oracle for arbitrary convex shapes or universal robust collision certification would require more than these bounded Float32 paths, particularly near coplanarity, tangency and tiny/deep intersections.

## 5. A hand-checkable collision fixture

The actual test [`GJK and EPA resolve convex penetration deterministically`](../tests/PhysicsEngineTests.cpp) uses a cube centered at `(0,0,0)` with half-extent `0.5` and a sphere of radius `0.5` centered at `(0.75,0,0)`.

Their nearest surfaces along X overlap by

```text
cube right face: x = +0.5
sphere left edge: x = 0.75 - 0.5 = +0.25
overlap depth: 0.5 - 0.25 = 0.25
expected A->B normal: approximately (+1,0,0)
```

The test asserts depth around 0.25 within a 0.002 margin and nearly +X normal. Repeating the same calculation checks deterministic iteration counts and depth for that fixture. Moving the sphere to `x=2` checks separation. Another fixture rotates a hull, reverses A/B, and expects a similar depth but approximately opposite normal.

Why this matters: a GJK/EPA implementation can report overlap with the wrong sign on the normal, causing the impulse solver to accelerate bodies *into* one another. A meaningful contact test checks not just `has_value()`, but **depth, normal sign and swapped-pair symmetry**.

The separate [narrowphase fixture](../tests/PhysicsEngineTests.cpp) checks rotated convex hull against an infinite plane, which is deliberately outside finite-support GJK. Another world-level fixture checks that the same hull participates in `Step`, `Raycast`, `SweepSphere` and debug geometry. That does not mean each query is implemented through the same algorithm.

### Supplement: boxes and triangle meshes do not all use GJK

The [`Narrowphase.cppm`](../Narrowphase.cppm) code includes specialized shape contact paths. An oriented box (OBB) against another OBB uses the **separating-axis theorem (SAT)**. For an axis `n`, the support radius of an OBB with unit axes `u_i` and half-extents `e_i` is:

```text
r_box(n) = sum_i abs(dot(n, u_i)) * e_i
overlap(n) = r_A(n) + r_B(n) - abs(dot(center_B-center_A,n))
```

When any candidate axis has nonpositive overlap, the boxes separate. A general OBB pair checks up to **15 axes**: three from the first box, three from the second, and nine edge-direction cross products. Tiny cross products are ignored because two nearly parallel edges do not yield a numerically trustworthy normal. The axis of minimum positive overlap suggests a response normal, but a solver also needs contact locations.

[`BuildBoxBoxContacts`](../Narrowphase.cppm) gathers vertices of A inside B and B inside A, shifts witnesses toward the overlap midpoint, removes nearly coincident points, and retains at most four contacts using a deepest-first then farthest-spread selection. If neither box contributes an interior vertex, it uses a fallback pair of support points. That is a practical limited manifold heuristic, **not an exact intersection-polygon clipper**, particularly for edge–edge contacts.

Sphere/box and capsule/box functions first compute a nearest-point pair: `ClosestPointOnBox` projects and clamps onto box axes; `ClosestPointsOnSegments` handles degenerate/parallel axial segments; and `ClosestSegmentBoxPoints` approximates a closest capsule-axis point using a bounded one-dimensional ternary search. When the sphere center is *inside* the OBB, `ClosestPointOnBox` returns that same interior point, so the normal cannot be computed by dividing a zero delta. `SphereBoxFallbackNormal` instead chooses the nearest box face direction. This is an explicit degeneracy policy and should be tested alongside shallow/centered overlap.

Triangle meshes have a separate *coordinate-space* problem. Mesh triangles and BVH acceleration nodes are indexed in collider-local space, while bodies and peer AABBs are usually in world space. [`WorldBoundsToColliderLocal`](../Narrowphase.cppm) transforms all eight corners of a world AABB with the inverse collider pose and expands a **conservative local AABB** for BVH queries. Passing untransformed world bounds to a local BVH would miss rotated/transformed geometry.

After finding candidate triangle IDs, `MakeTriangleCollisionPrism` extrudes each triangle by `±SurfaceThickness/2` into a closed six-vertex convex prism. A finite convex peer can then use `CollideConvex` on this proxy; mesh-versus-mesh checks two sets of BVH candidates and can retain only the deepest representative contact. This is an **approximate thin-surface collision treatment**, not a continuous/deformable mesh solver. The triangle prism thickness changes when contacts may be reported; it is a simulation parameter, not just a visual appearance setting.

The key call graph is therefore:

```text
Broadphase candidate pair
    -> Narrowphase::CollidePair
       -> box vs box: FindBoxBoxSAT -> BuildBoxBoxContacts
       -> capsule/sphere vs box: closest witnesses + explicit normal fallback
       -> convex vs convex: CollideConvex (GJK then EPA)
       -> mesh vs convex: local BVH candidate query -> triangle prism -> CollideConvex
       -> mesh vs mesh: two local BVH candidate queries -> prisms -> representative hit
       -> plane: dedicated infinite-geometry branch
    -> ContactManifold -> PhysicsWorld policy -> solver
```

**Exercises:** rotate one box until one edge-cross-product axis becomes necessary for separation; confirm `FindBoxBoxSAT` tests it. Place a sphere center exactly inside an OBB and inspect fallback normal and penetration depth. Rotate a triangle mesh collider and ensure local BVH queries still retrieve its candidate triangles. Compare physically meaningful contact density with the four-point manifold cap.

## 6. Raycasts and sphere-sweep distance are separate operations

[`RaycastConvexHull`](../ConvexCollision.cppm) clips a ray against each outward face half-space. An entering face can increase the earliest hit distance, and an exiting face can decrease the latest allowed distance. If entry exceeds exit, no ray–polytope intersection remains. A nearly parallel ray outside a face fails immediately.

[`PointConvexHullSeparation`](../ConvexCollision.cppm) instead checks whether a world point lies inside all hull face half-spaces, then finds its nearest surface triangle point when outside. It returns unsigned distance and a direction used by sphere-sweep reasoning.

Neither operation is a contact impulse. A ray hit says where a ray intersects geometry; a sweep asks when a displaced radius might reach geometry; the constraint solver takes contact manifolds and changes body momenta. Keep their result types and assertions distinct.

## 7. Why snapshots must contain more than transforms

A visible `position` and `rotation` do not define a complete ongoing simulation. [`PhysicsWorld::CaptureSnapshot`](../PhysicsWorld.cppm) also stores velocities, body/collider/joint topology, collision-layer/pair rules, previous contact identity, warm-started contact impulses and the fixed-step accumulator.

[`RestoreSnapshot`](../PhysicsWorld.cppm) validates the complete snapshot before replacing live arrays, rebuilds collider metadata and spatial proxies, then reconstructs solver island state. Host callbacks and wall-clock profiler values are intentionally outside that deterministic state.

A snapshot is therefore a **state checkpoint**. A replay is different: it begins from a snapshot and records the *external commands between deterministic fixed steps*. Saving a screenshot or only body transforms would lose important information needed to reproduce resting contact transitions.

## 8. Replay executes a recorded command protocol

[`PhysicsReplayRecorder`](../PhysicsReplay.cppm) owns `InitialState`, `FixedDt` and `Frames`. For each frame:

```text
commands = [external force/torque/impulse/wake/CCD/motion mutations]
for command in commands:
    ApplyPhysicsReplayCommand(world, command)
world.Step(FixedDt)
record(ExpectedStateHash = PhysicsStateHash(world))
```

[`VerifyPhysicsReplay`](../PhysicsReplay.cppm) restores the original world snapshot and repeats that sequence. It compares actual hash after each step and returns at the **first mismatch**, recording its zero-based frame index and expected/actual values.

The corresponding test records 40 steps at 1/120 second with force applied over the first ten frames and one impulse at frame twelve. It encodes/decodes the replay, verifies all 40 frames, and corrupts the stored hash of frame five to require `DivergentFrame == 5`. This tests that the reporting mechanism identifies a mismatch; it does not prove any particular solver has no physics error.

Hash agreement is useful regression evidence for a **known code build and execution environment**. Floating point reduction order, hardware instructions, task scheduling and algorithm changes can cause real divergence across environments. A hash collision is also theoretically possible, so it is not a formal proof that all state values are identical.

## 9. Replay binary format, versioning and I/O failure modes

[`SerializePhysicsReplay`](../PhysicsReplay.cppm) writes a versioned `KPREP01\n` magic header, fixed dt, length-prefixed serialized initial snapshot, frame counts, tagged commands and expected hashes.

Command tags currently represent:

| Tag | Command | Authoritative mutation |
| --- | --- | --- |
| 0 | add force | affects accumulated force |
| 1 | add torque | affects accumulated torque |
| 2 | impulse at point | changes linear/angular momentum |
| 3 | set motion state | assigns pose/velocities with finite-value checks |
| 4 | wake body | changes sleep/active-motion state |
| 5 | set CCD mode | selects discrete/continuous motion policy |

`writeCommand` and `readCommand` must remain synchronized. Introducing another command is a **wire-format compatibility change**, not merely extending a C++ variant; old binaries must either understand the new version or fail clearly.

The byte reader bounds-checks offsets, decoded frame count and commands per frame; it also rejects wrong magic/version and trailing data. The file loader refuses source files over the declared size limit before allocating. The file writer stages `.tmp`, uses a `.bak` for previous destination, and renames the new version into place with a rollback attempt on error.

This is good error-handling intent, but do not claim fully crash-proof persistence: atomic rename and durable disk flush are not the same on every filesystem, and sidecar file cleanup deserves fault-injection testing.

## 10. Build and run the actual available test target

The repository's [`CMakeLists.txt`](../CMakeLists.txt) declares `KairoPhysicsEngineTests`, which includes convex and replay cases in [`PhysicsEngineTests.cpp`](../tests/PhysicsEngineTests.cpp). On a host that can compile the C++23 module graph and has its required dependencies:

```bash
cmake -S . -B build -G Ninja \
  -DKAIRO_PHYSICS_ENGINE_BUILD_TESTS=ON
cmake --build build --target KairoPhysicsEngineTests --parallel 4
ctest --test-dir build -R KairoPhysicsEngineTests --output-on-failure
```

Catch2 tags visible in source include `[PhysicsEngine][Convex][GJK][EPA]`, `[PhysicsEngine][Convex][Narrowphase]`, `[PhysicsEngine][Convex][World]`, `[PhysicsEngine][Replay]` and `[PhysicsEngine][Serialization][Validation]`. The stress test repeats a 36-sphere, 360-step world and checks final positions within `1e-6` on the same host. It is not a controlled performance benchmark.

**Current pass verification:** these source files and test definitions were inspected and annotated; no native build/CTest or performance trial was executed here.

## 11. Targeted exercises for the next implementation/check pass

**Geometry:** manually compute sphere-vs-cube support in +X and −X; draw line/triangle/tetrahedron simplex origin regions; test co-centered shapes, exact tangency and deeply overlapping hulls. Record every `GJKIterations` and `EPAIterations`, including `nullopt`, to distinguish separation from failed convergence.

**Contact identity:** verify that reversing collider order reverses A→B normal without changing depth; attach two colliders to one body and confirm broadphase candidate filtering doesn't create same-body self-impulses. Ensure a trigger still creates events but not solver velocity changes.

**Replay:** flip a hash at an intermediate frame, delete a payload byte, append extra bytes, supply an unknown command tag, corrupt the snapshot body IDs, and simulate an interrupted staged write. Verify that load failure preserves a previously loaded world. Compare two repeated local runs against the same fixed source revision and then intentionally vary the world settings to observe hash divergence.

**Performance:** instrument the GJK/EPA iteration distribution and nullopt reasons before comparing to an alternative broadphase/penetration algorithm. Keep broadphase time, narrowphase time, solver time, numerical correctness and end-to-end frame time separate. A faster candidate-generation algorithm does not establish physically correct contact normals or more stable dynamics.

If a question cannot be answered from the code or test assertion, treat it as an **open verification task** instead of making it part of the documented contract.
