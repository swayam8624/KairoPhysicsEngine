# KairoPhysicsEngine V1 Stability Contract

KairoPhysicsEngine V1 is the stable rigid-body simulation layer for Kairo.

## Supported body model

- Static, kinematic, and dynamic rigid bodies.
- Authored mass and inertia properties for dynamic bodies.
- Gravity, damping, linear/angular velocity limits, sleeping, waking, forces, torques, impulses, and fixed stepping.
- Per-body discrete or continuous collision detection.

## Supported collision geometry

- Sphere.
- Capsule.
- Plane.
- Axis-aligned box.
- Oriented box.
- Validated convex hull.
- Triangle mesh on static, kinematic, or dynamic bodies.

Triangle meshes keep their acceleration structure in collider-local space. Body and collider transforms are applied when broadphase bounds, queries, sweeps, and narrowphase contacts are evaluated, so moving meshes do not rebuild their local BVH every frame.

Dynamic triangle-mesh bodies use the mass and inertia supplied by the body description. The collision mesh does not implicitly derive mass properties.

## Collision pipeline

- Dynamic AABB-tree broadphase with deterministic pair ordering.
- GJK/EPA finite-convex narrowphase.
- SAT/contact generation for box cases where appropriate.
- BVH-accelerated triangle-mesh queries and contacts.
- BVH-accelerated mesh-to-mesh candidate reduction followed by convex triangle-prism contact testing.
- Persistent contact caching and warm starting.
- Collision layers, masks, pair overrides, triggers, and contact lifecycle events.

## Continuous collision detection

V1 continuous collision detection is conservative for translational tunnelling. Finite moving shapes use a sweep bound; triangle meshes use the maximum local vertex radius as their moving-source sweep radius. Triangle meshes remain exact accelerated targets for sphere sweeps.

This can stop boxes, capsules, hulls, and triangle meshes slightly early because the moving source is conservatively bounded. Angular swept-volume CCD is not part of the V1 guarantee; fast rotation should use smaller fixed steps or a conservative authored collision proxy.

## Constraints and simulation

- Distance joints.
- Ball-socket joints.
- Fixed joints.
- Hinge joints.
- Deterministic solver-island construction.
- Optional parallel island solving where independent islands permit it.
- Sequential-impulse contact solving with friction and restitution.

## Queries and gameplay systems

- AABB and sphere overlap queries.
- Nearest and all-hit raycasts.
- Sphere sweeps.
- Projectile simulation for hitscan and ballistic use cases.
- Buoyancy volumes.
- Physics debug-shape extraction.

## Persistence and reproducibility

- Versioned world snapshots.
- Snapshot serialization/deserialization and file IO.
- Deterministic state hashing.
- Command-driven replay verification and divergence reporting.

V1 replay guarantees apply to the replay command surface encoded by the current replay format. Structural world edits made outside that command stream must be represented in the initial snapshot or in a later replay-format extension.

## Stability rule

Changes that alter public rigid-body behavior, snapshot/replay compatibility, contact ordering, or deterministic results require regression coverage. The supported V1 surface must build and pass tests under the repository CI matrix on Ubuntu/Clang, macOS/LLVM, and Windows/MSVC before release or integration into the KairoGameEngine superproject.
