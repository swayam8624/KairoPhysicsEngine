# KairoPhysicsEngine

`KairoPhysicsEngine` is the rigid-body simulation layer built on top of:

```text
KairoMath -> KairoPhysicsMath -> KairoGeometry -> KairoSpatial -> KairoPhysicsEngine
```

This repo owns simulation concepts such as rigid bodies, colliders, broadphase
pairs, narrowphase contacts, contact solving, stepping, and debug extraction.
It is not a renderer, editor, ImGui tool, or sandbox viewer.

## Build

```bash
cd /Users/swayamsingal/Desktop/Programming/Kairo/Foundation/KairoPhysicsEngine
cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++
cmake --build build
ctest --test-dir build --output-on-failure
```

Dependencies resolve local Kairo repos first:

```text
KAIRO_PHYSICS_MATH_SOURCE_DIR -> ../KairoPhysicsMath -> GitHub
KAIRO_SPATIAL_SOURCE_DIR      -> ../Spatial          -> GitHub
```

## V1 Boundary

V1 will focus on CPU rigid-body simulation:

```text
RigidBody records
Sphere, plane, and AABB colliders
Dynamic AABB tree broadphase through KairoSpatial
Sphere-sphere, sphere-plane, and AABB-AABB narrowphase
Sequential impulse contact solving
Fixed-step PhysicsWorld
Debug contacts and AABBs
Deterministic tests
```

No visualization or full engine/editor systems belong in this repo yet.
