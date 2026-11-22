# PhysicsWorld snapshot files: binary format, ownership and strong restore guarantees

This is an advanced C++23 source-reading chapter for [`PhysicsSerialization.cppm`](../PhysicsSerialization.cppm), [`PhysicsWorld.cppm`](../PhysicsWorld.cppm) and [`PhysicsReplay.cppm`](../PhysicsReplay.cppm), extending the [GJK, EPA and replay laboratory](CONVEX_COLLISION_AND_REPLAY_FROM_FIRST_PRINCIPLES.md).

A game engine must separate three things: the *visible frame*, the *authoritative physics state*, and the *serialized representation*. A drawing may show one sphere at `(1,2,3)` while the actual next physics step also depends on its linear/angular velocities, mass, damping, sleep timer, accumulated force and impulse cache. A replay adds a fourth layer: a record of **which external mutations occurred between successive steps**.

## 1. Who owns what?

`PhysicsWorld` is the authoritative owner of vectors of `RigidBody`, `Collider` and `Joint` records. IDs correspond to stable vector slots. The broadphase owns a cache of spatial proxies derived from those records, not the records themselves; the solver owns no independent world.

`CaptureSnapshot()` returns an **owned copy** of settings, gravity, body/collider/joint data, prior broadphase pairs, manifolds, collision-response overrides, previous contact keys, cached impulses and the fractional fixed-step accumulator. It omits host callback functions and wall-clock profiling.

`RestoreSnapshot(snapshot)` validates version, finite values, ID/index consistency, references, shape constraints and other invariants **before replacing live data**; then reconstructs derived collider metadata, broadphase proxies and solver islands.

`SerializePhysicsWorldSnapshot(snapshot)` converts that snapshot to byte fields; `DeserializePhysicsWorldSnapshot(bytes)` reconstructs *raw snapshot records*, but decoding alone does not prove cross-record semantic validity. The public `LoadPhysicsWorld(path, world)` does a separate throwaway-world restoration test before mutating the destination.

## 2. What the binary header means

The current source defines two version markers:

```text
8-byte magic:  K P H Y S 0 1 newline
file version:  1 (uint32 little-endian)
snapshot data: fixed-order typed fields
```

`PhysicsSnapshotFileVersion` identifies the binary wrapper format. `PhysicsWorldSnapshotVersion` identifies the in-memory simulation-state schema validated by `RestoreSnapshot`. Do not assume these two versions must advance together; changing any field order or tag assignment of the wire schema requires explicit backwards-compatibility decisions.

The [`Writer`](../PhysicsSerialization.cppm) uses little-endian integer emission and `std::bit_cast<std::uint32_t>` for each float. There are no decimal strings to parse or locale-dependent number formats. This is an **exact bit-representation serialization**, not a portable canonical numerical representation for every possible architecture.

The [`Reader`](../PhysicsSerialization.cppm) checks each required byte count before incrementing its cursor, rejects encoded booleans other than 0/1 and rejects any trailing data after a complete record. A validated byte buffer may still encode an invalid body ID, nonunit rotation or geometrically invalid hull; those are separate semantic checks.

## 3. Know the data schema from the write order

The serializer writes fields in an intentional sequence:

| Section | What is preserved | Why the next step needs it |
| --- | --- | --- |
| World policy | solver iterations, gravity, slop, sleep settings, accumulator | the next fixed step must follow the same policy and elapsed-time schedule |
| Bodies | IDs, active state, type, motion, mass/inverse inertia, accumulated forces, damping, limits, CCD, sleeping | velocities, torque and sleep affect next movement |
| Colliders | IDs, body references, local transforms, shape variant, material, filters and triggers | contacts require exact authored shape and response policy |
| Joints | IDs, references, type-specific anchors/rotations/axes | island partition and joint constraints depend on topology |
| Prior pairs/manifolds | collider pairs and contact points including cached impulses | collision continuity and solver warm start |
| Response rules | pair and layer Ignore/Trigger/Block overrides | geometry alone does not decide whether collision is blocking |
| Event keys | previous contact identity and trigger state | Begin/Stay/End depends on prior-step contact existence |
| Contact cache | local anchors, contact normal, normal/tangent impulse | convergence can change if warm-start state is lost |

For a collider shape variant, the binary tags currently distinguish a sphere, capsule, plane, AABB, oriented box, convex hull and triangle mesh. Convex hull/mesh variants include bounded vertex counts and face/triangle index triples, not just an image of their outline. The decoder returns the authored data; `RestoreSnapshot` must rebuild validation-sensitive collider metadata before use.

The contact manifold writer caps each manifold at 64 serialized points, even though typical solver manifolds are reduced to fewer contacts. This is a serialization limit; it is not proof of a 64-point solver policy.

## 4. Why a malformed file must fail without corrupting the world

The public [`LoadPhysicsWorld`](../PhysicsSerialization.cppm) follows the logical sequence:

```text
ReadFile(path)
   -> stat and reject over-limit size
   -> allocate bounded buffer, read full byte count
DeserializePhysicsWorldSnapshot(bytes)
   -> require magic/version
   -> decode bounded arrays with known tags
   -> require exact end of input
PhysicsWorld validationTarget
validationTarget.RestoreSnapshot(snapshot)   [full semantic validation]
destinationWorld.RestoreSnapshot(snapshot)   [replace live state]
```

This **validate-then-commit** structure is preferable to mutating live objects record by record during a parse. The existing [Catch2 test](../tests/PhysicsEngineTests.cpp) saves a valid world, loads it, corrupts the file, requires a thrown exception and asserts the previously loaded world's state hash has not changed. That is a good targeted **strong-exception-safety test for malformed input**.

The world also preserves host callbacks through restoration because callback function objects are process-owned. Replaying a saved world cannot recreate arbitrary open file handles, GPU resources, audio voices, operating system timers or scripting VM closures. Those systems require their own lifecycle semantics.

## 5. Raw byte validity is not the same as world validity

The serialized `RigidBody` structure includes a numeric ID and motion state, but arbitrary ID values could be physically meaningless or refer to missing records. Similarly, a hull may contain out-of-range vertex indices, a non-closed face set or invalid convex geometry even if all its bytes parse correctly.

That is why `DeserializePhysicsWorldSnapshot` returns a **data record** and the replay/loader ultimately uses `PhysicsWorld::RestoreSnapshot` to enforce world-specific invariants. Some checks can be local (`bool` tags, array caps), while others require the entire set of records (a collider refers to an active body, a joint references two valid bodies, a previous-contact key references an existing collider).

When designing a new save file format, ask: should a rule be checked by the byte reader, by object construction, or by a final transaction validator? The answer determines whether a malformed file can cause unsafe partial state.

## 6. A staged write is not automatically crash-proof

The writer constructs a temporary file with suffix `.tmp`, optionally renames the old destination to `.bak`, renames the temporary file to the final name, then removes the backup. If the publish rename fails, it attempts to restore the old file.

This reduces the risk of overwriting a valid file with an incompletely written stream. But staged renames are not by themselves **durable fsync guarantees**; power loss, filesystem metadata timing, backup cleanup errors, permissions and cross-device behavior require testing separately.

A particularly useful fault-injection matrix exercises write failures before/after flush, failure while staging the old file, failure publishing the temporary file, and failure deleting the backup. For each case, record which paths remain, whether the previously saved world can be loaded, and whether a recovery procedure is documented.

## 7. Why replay state hashes use FNV rather than a cryptographic proof

The current [`PhysicsStateHash`](../PhysicsSerialization.cppm) computes a 64-bit FNV-1a digest of the serialized world snapshot bytes. Every change to included body velocity, contact cache, response rule or other serialized field can change the digest.

The replay module records one expected hash after each fixed-step command batch. Matching hashes across repeated runs are useful to detect regressions **for the same implementation and relevant numerical environment**. FNV-1a is not cryptographically collision-resistant, does not authenticate a file and is not by itself evidence that a solver obeys conservation laws. A simulation can faithfully replay an incorrect algorithm.

Also, this hash excludes host callbacks and profiling clocks because those are not part of the serializable deterministic simulation state. The hash therefore cannot detect a gameplay callback misfire or rendering mismatch unless those effects change an included authoritative field.

## 8. Serialization caps and memory safety

The outer snapshot format has a 256 MiB file size limit; individual record counts and shape element counts have separate caps. `Writer::Count` checks bounds before encoding a 32-bit count, and `Writer::Raw` checks capacity before bulk appends. `Reader::Count` checks the corresponding maximum before reserving vectors.

**Review nuance:** the simple scalar `Writer::U8/U32/U64` append primitives do not individually enforce the outer byte budget. A complete writer correctness audit should therefore test aggregate output size—not only individual `Raw` calls—and ensure a final encoded buffer cannot silently exceed the advertised maximum under extreme but count-valid structures.

Limits are valuable because valid-looking length prefixes can otherwise request unbounded allocation before the parser reaches a malformed field. They do not eliminate all arithmetic-overflow, memory-pressure or platform-error concerns; those need dedicated hostile-input tests.

## 9. Concrete verification exercises

**Exercise A — save/restore correctness.** Build a PhysicsWorld with two bodies and two colliders, one explicit collision-response override and a joint. Step several times, capture a snapshot and serialize it. Restore it into another PhysicsWorld; compare `PhysicsStateHash` before and after and then step both worlds one more fixed interval to check numerical continuity on the same host.

**Exercise B — malformed data safety.** Start from a valid snapshot buffer. Remove the final byte, append an extra byte, change the magic, change the version, change the first body ID to an index that cannot exist, and corrupt a collider's authored hull faces. Separate **decode errors** from **semantic restore errors** and require the destination world to stay unchanged.

**Exercise C — save-file failure injection.** Force a failure at each staged-write boundary. Confirm that the old snapshot stays valid or that the API reports a recoverable state with explicit surviving files. Do not assume temporary files are automatically deleted on sudden power loss.

**Exercise D — replay parity.** Use [`PhysicsReplayRecorder`](../PhysicsReplay.cppm) to record commands and `VerifyPhysicsReplay` to replay. Intentionally tamper with the expected hash of one frame and verify the exact zero-based divergence index. Then change a body material or solver setting and observe which serialized state fields/step hashes change.

**Exercise E — portability.** Compile/run on distinct compiler versions or CPU architectures where available. Record FP contraction/fast-math flags, thread/island settings, exact commit and test output. If hashes differ, compare canonical decoded state fields to establish whether the difference is harmless FP rounding, altered solver ordering, or a real logic discrepancy.

## 10. Run the existing native test target

The repository registers `KairoPhysicsEngineTests`. On a host with C++23 module support and the pinned KairoMath/Geometry/Spatial dependencies available:

```bash
cmake -S . -B build -G Ninja \
  -DKAIRO_PHYSICS_ENGINE_BUILD_TESTS=ON
cmake --build build --parallel 4 \
  --target KairoPhysicsEngineTests
ctest --test-dir build -R KairoPhysicsEngineTests --output-on-failure
```

Inspect [`tests/PhysicsEngineTests.cpp`](../tests/PhysicsEngineTests.cpp) for `[PhysicsEngine][Serialization][File]`, `[PhysicsEngine][Serialization][Validation]` and `[PhysicsEngine][Replay]` cases. Their assertions are source-visible; this documentation pass did not execute them or certify the exact-head build.

**Study principle:** a parseable file is not necessarily a valid scene, a loaded world is not automatically a visually identical frame, and a repeated hash does not establish physics correctness. Each layer needs its own invariant and test evidence.
