# Physics Snapshot And Replay Contract

KairoPhysicsEngine snapshots are deterministic simulation-state records, not host-process dumps.
They preserve the state required to continue a fixed-step rigid-body simulation with the same
solver conditioning while rebuilding derived acceleration structures on restore.

## Snapshot ownership

A snapshot includes step settings, gravity, bodies, colliders, joints, fixed-step accumulator,
collision response rules, previous-contact identity, current broadphase/contact records, and the
persistent warm-start cache. Triangle-mesh authored geometry is serialized; its BVH is rebuilt
through the validated collider construction path after load.

Wall-clock profiling and host callbacks are deliberately excluded. They are process services,
not deterministic simulation state.

## Binary format

`SerializePhysicsWorldSnapshot` emits the bounded version-1 binary format. All scalar fields use
explicit little-endian encoding and floating-point values are serialized by their IEEE-754 bits.
The reader rejects unsupported versions, invalid enum tags, excessive record counts, truncation,
corrupt shape/joint payloads, and trailing bytes.

`SavePhysicsWorld` publishes through a temporary file and `LoadPhysicsWorld` parses and validates
the complete replacement before mutating the destination world.

## State hashes

`PhysicsStateHash` hashes the canonical serialized snapshot. It is intended for deterministic
regression evidence and replay divergence detection; it is not a cryptographic authentication
primitive.

## Replay

A replay owns an initial snapshot, one fixed timestep, and an ordered frame list. Each frame
contains deterministic physics commands followed by the expected post-step state hash.
Supported version-1 commands cover force, torque, point impulse, motion-state replacement, waking,
and collision-detection-mode changes.

`VerifyPhysicsReplay` restores the initial state, reapplies every command in order, advances one
fixed step per frame, and reports the first frame whose state hash diverges.

Structural body/collider/joint creation or destruction during recording is intentionally outside
version 1. Author those structures before capture; a future replay version can add explicit
structural commands without changing the meaning of existing files.
