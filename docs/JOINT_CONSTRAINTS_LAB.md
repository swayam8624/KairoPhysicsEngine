# Joint constraints from first principles: anchors, Jacobians, islands and numerical stability

This is a guided read through [`Joint.cppm`](../Joint.cppm), [`SolverIsland.cppm`](../SolverIsland.cppm), [`PhysicsWorld.cppm`](../PhysicsWorld.cppm), and the actual [Catch2 joint tests](../tests/PhysicsEngineTests.cpp). Read the earlier [contact solver mathematics](CONTACT_SOLVER_FROM_EQUATIONS_TO_CODE.md) and [convex contact geometry laboratory](CONVEX_COLLISION_AND_REPLAY_FROM_FIRST_PRINCIPLES.md) first. This chapter joins their output to jointed-body dynamics and KairoPhysicsMath's [`Impulse.cppm`](https://github.com/swayam8624/KairoPhysicsMath/blob/main/Impulse.cppm), [`RigidBodyMath.cppm`](https://github.com/swayam8624/KairoPhysicsMath/blob/main/RigidBodyMath.cppm), and [`ConstraintMath.cppm`](https://github.com/swayam8624/KairoPhysicsMath/blob/main/ConstraintMath.cppm).

**Verification status:** code and test sources were inspected, teaching comments added, and this guide committed. No current-host build, native simulation, or sanitizer run was performed during this documentation pass.

## 1. A joint constrains relative motion; it does not own either body

`Joint` is a value record with a stable `JointID`, two `BodyID` references, an `Active` flag, a `CollideConnected` policy bit, and a tagged `JointConstraint` variant. `PhysicsWorld` owns its stored bodies and joints; `SolverIsland` refers to their **indices** when grouping work. The `Joint` itself owns neither `RigidBody` instance nor a render transform.

Local joint anchors are in each body's coordinate system, not fixed world points. When the body changes orientation, the anchor world position is recomputed:

```text
anchorWorld = body.Position + Rotate(body.Rotation, localAnchor)
```

If a developer naively saves an anchor's world position when constructing the joint and reuses it forever, a rotating rigid body would no longer be constrained at its physical attachment point. This is why [`WorldJointAnchor`](../Joint.cppm) is called inside velocity and positional solver stages.

The public constructors `MakeDistanceJoint`, `MakeBallSocketJoint`, `MakeFixedJoint`, and `MakeHingeJoint` validate local inputs; `PhysicsWorld::Create*Joint` verifies body identities and stores a new slot. `IsActiveJoint` merely tests structural fields—valid distinct IDs and activity—not whether those IDs still refer to active records in a particular world. The solve methods perform the remaining checks.

## 2. Four constraints and their remaining degrees of freedom

| Joint type | Enforced condition | Remaining freedom |
| --- | --- | --- |
| Distance | Distance between world anchors equals `RestLength` | Moving on a sphere about the anchor, plus relative orientation |
| Ball socket | World anchors coincide, three translational equations | All three relative rotational axes |
| Fixed | World anchors coincide **and** B's rotation matches A times its reference rotation | Ideally none of the six relative degrees of freedom |
| Hinge | World anchors coincide and hinge axes align | Rotation about the shared hinge axis |

None of these four current variants defines an articulated motor controller, angular stops, a breakage threshold, or a spring compliance parameter. A future implementation can add them, but the current `HingeJoint` should not be advertised as a complete motorized robotics joint.

The `CollideConnected` bit controls contact filtering for the two bodies; it is not an instruction to the joint solver to apply an extra impulse. Keeping contact policy separate from the joint equation prevents double-solving the same physical restriction.

## 3. Why a contact Jacobian also solves a joint

Consider a world-space unit axis `n` and attachment offsets from each body's center of mass:

```text
rA = worldAnchorA - bodyA.Position
rB = worldAnchorB - bodyB.Position

vA_contact = vA_COM + omegaA × rA
vB_contact = vB_COM + omegaB × rB

relativeAxisSpeed = dot(vB_contact - vA_contact, n)
positionError = dot(worldAnchorB - worldAnchorA, n)
```

The velocity-level joint objective is approximately

```text
relativeAxisSpeed + bias ≈ 0

bias = clamp(beta * positionError / dt,
             -maxPositionCorrection / dt,
              maxPositionCorrection / dt)
```

where `beta` is the configured `Baumgarte` stabilization coefficient. If the anchors are too far apart along `n`, the bias requests a velocity correction bringing them together.

The scalar effective inverse mass, taken from `PhysicsMath::ContactNormalDenominator`, is

```text
D = inverseMassA + inverseMassB
  + dot(n, (inverseInertiaWorldA * (rA × n)) × rA)
  + dot(n, (inverseInertiaWorldB * (rB × n)) × rB)
```

For `D` near zero the row is unsolvable or both bodies are locked, so [`ApplyLinearAxisConstraint`](../Joint.cppm) skips it. Otherwise the bilateral impulse magnitude is

```text
lambda = -(relativeAxisSpeed + bias) / D
impulse = lambda * n
```

The impulses are opposite for the two bodies: A receives `-impulse`, B receives `+impulse`, at their respective world anchors. The point-impulse helper updates both linear velocity `v += inverseMass * impulse` and angular velocity `omega += inverseInertiaWorld * (r × impulse)`.

This is what makes a joint feel like an attachment rather than an arbitrary positional teleport: the constraint reacts to the body's mass distribution and the lever arm. An off-center joint can induce rotation even when its constraint error is a linear displacement.

**Important difference from contact:** a contact's normal impulse typically cannot become tensile (negative accumulated normal impulse would pull objects together). A distance or point joint is **bilateral**: it may pull or push to preserve the authored relationship.

## 4. Hand-computing one constrained step

Take body A as static and body B as a 1 kg translating point body with negligible rotational effect. Both anchors begin one meter apart along world X, but the joint wants `RestLength=1.5 m`. Suppose B currently moves away from A at `+2 m/s`, the step uses `dt=1/60 s`, and stabilization is `beta=0.2`.

Choose `n=+X`. The signed position error is `1-1.5=-0.5 m`; the normal velocity is `+2 m/s`; and the effective denominator is `D=1 kg⁻¹`.

```text
unclampedBias = 0.2 * (-0.5) / (1/60) = -6 m/s
lambda = -(2 + (-6)) / 1 = +4 kg*m/s
vB_after = 2 + 4 = 6 m/s
```

Why does this appear counterintuitive? In this code, the positional-error sign and the solver's axis are coupled to the exact implementation choice: `axis` is normalized from anchor A toward anchor B, and the desired rest length is **larger**, so the correct correction is to push B further outward. A positive impulse in +X does precisely that. The current `+2 m/s` motion is not enough to fix a half-meter shortage within one frame, so stabilization requests more outward speed. When distance exceeds its target, the signed error changes sign, the impulse becomes negative, and the joint pulls bodies together.

Now place the joint away from B's center of mass and use its nonzero inertia tensor. The angular contribution increases `D`, reducing the same scalar impulse magnitude and distributing the correction between translational and rotational motion. Static/kinematic anchor bodies contribute zero inverse mass for solving.

The engine also clamps the bias by `MaxPositionCorrection / dt`, limits iterations, and performs a separate bounded positional correction. Real-world large-error motion therefore differs from the unbounded textbook scalar calculation.

## 5. Ball socket = three sequential world-axis rows

A ball-socket constraint is conceptually a vector equation:

```text
C = worldAnchorB - worldAnchorA = (Cx,Cy,Cz) = (0,0,0)
```

[`ApplyPointConstraint`](../Joint.cppm) enforces one X, one Y, then one Z velocity row by calling `ApplyLinearAxisConstraint`. It does **not** solve a single coupled 3×3 matrix equation. Earlier rows can change angular and linear velocities used by later rows, so iteration count and numerical conditioning matter.

An ideal unconstrained ball socket allows arbitrary relative orientation while keeping its two anchors together. The implementation tests that condition using the **world-space anchors**, not by comparing two bodies' origins.

## 6. Fixed joint orientation comes from a relative quaternion

A fixed joint stores `ReferenceRotation` alongside its local anchors. The intended target rotation is:

```text
desiredRotationB = normalize(rotationA * ReferenceRotation)
errorQuat = normalize(inverse(desiredRotationB) * rotationB)
```

[`FixedJointAngularError`](../Joint.cppm) selects the shorter quaternion branch by negating an error quaternion whose scalar part is negative. It converts the error into a world-space axis–angle vector. The current velocity stage then applies angular constraints along world X, Y and Z.

The quaternion is not subtracted component-by-component: `qB - qExpected` would not be a valid rotation error measure. A finite 3D rotational error must account for quaternion composition and periodicity.

In the positional correction stage, an axis-angle correction is limited and distributed between bodies according to inverse-angular-inertia mobility along the relevant axis. `ApplyOrientationDelta` composes a small axis-angle quaternion with the current body orientation and normalizes the result.

## 7. Hinge alignment and the difficult 180-degree case

A hinge should allow rotation about **one common world axis**. Each body's local axis is rotated to world space and normalized. The solver computes an average hinge axis and constructs two perpendicular tangent directions around it. It then suppresses angular error along those tangents, but does **not** constrain angular speed along the hinge axis.

[`AxisAlignmentError`](../Joint.cppm) uses the cross product of normalized axes to find a correcting direction and `atan2(sinAngle,cosAngle)` to obtain the angle. Parallel vectors have zero cross product and need no correction. **Antiparallel vectors also have zero cross product** but differ by 180 degrees; the implementation deliberately chooses a perpendicular fallback axis and returns an angle of π radians. This is a common source of hinge bugs and deserves a dedicated test.

The test's free axis is Y: after a bounded run, it requires X and Z angular speeds to be small but Y speed to remain appreciable. That verifies *freedom* as well as *constraint*—two equally important sides of a hinge.

## 8. Velocity solve and position solve are separate on purpose

During `PhysicsWorld::Step`, the solver first restores contact warm-start data, solves per-island velocity rows for contacts and joints, stores contact cache, and then applies separate position projections. [`SolveJointVelocity`](../Joint.cppm) uses current relative velocities and bias, while [`CorrectJointPosition`](../Joint.cppm) directly moves or rotates dynamic bodies to reduce residual pose errors.

The translational projection clamps total correction to `MaxPositionCorrection`, divides it by `PositionIterations`, and distributes the offset based on inverse mass. The rotational projection similarly caps the axis-angle correction (currently 0.25 radians) and distributes it according to projected angular mobility.

These two correction phases are **approximate stabilization**, not a global simultaneous constrained mechanics solve. More velocity iterations may improve convergence but cost more CPU time; stronger positional bias may reduce drift but inject nonphysical energy; too few iterations can yield long-chain stretching or oscillation.

There is an important distinction in persistent state: the contact system stores accumulated impulse scalars across frames for warm-starting, but the inspected [`Joint.cppm`](../Joint.cppm) does **not** store a per-joint persistent impulse cache. This is a concrete extension opportunity when implementing stable long chains or advanced articulation.

## 9. Solver islands make parallelism possible, not automatic

[`BuildSolverIslands`](../SolverIsland.cppm) first uses a union–find structure over body IDs. Contact and joint edges join two **dynamic** bodies; a static anchor is not a dynamic union vertex. Every dynamic body becomes a member of exactly one island. Then contact and joint *indices* are attached to their dynamic endpoint's island.

Why not merge every pair touching the same static floor? That would turn a scene of 1,000 otherwise unrelated falling objects into a single giant island, defeating parallelism. Shared static bodies can appear in multiple islands because the velocity and positional solver does not write to them. Connecting two dynamic bodies with a joint or a real blocking contact correctly merges their islands.

[`DispatchIslands`](../SolverIsland.cppm) executes serially below a configured threshold, or uses `std::jthread` workers that atomically claim island indices. Thread safety depends on a structural invariant: separate islands must not mutate the same **dynamic body, contact manifold or joint record**. It does not arise merely because `std::jthread` automatically joins at scope exit. Any future constraint type that writes shared external state could invalidate this property.

The solver-island test has two dynamic spheres attached to the **same static anchor**: it expects two islands. Adding a joint between those two dynamic spheres then makes one island. A second test steps equivalent serial and parallel configurations 90 times and compares positions and velocities within `1e-4`. That is bounded same-test numerical agreement, not a TSAN report or proof of deterministic scheduling across operating systems.

## 10. What the current Catch2 tests actually assert

The six relevant named cases in [`tests/PhysicsEngineTests.cpp`](../tests/PhysicsEngineTests.cpp) are:

| Test | Current observed contract in source |
| --- | --- |
| `[Joint][Distance]` | 180 steps at 1/120 s, final separation within 0.04 m of 2 m |
| `[Joint][BallSocket]` | 180 steps, world anchors separated by less than 0.04 m |
| `[Joint][Fixed]` | 120 steps, anchored pose and quaternion components within approximately 0.04–0.05 |
| `[Joint][Hinge]` | 30 steps, X/Z angular speed below 0.5; free Y speed above 1 |
| `[Island]` | Static bridges don't merge islands; dynamic joint bridges do |
| `[Island][Parallel]` | 90 steps of serial/parallel results agree within 1e-4 on positions and velocities |

These are **existing source assertions**; they were not re-run in this documentation pass. They establish useful examples but do not exhaust failure cases.

## 11. Run and extend the source-visible tests

The [CMakeLists.txt](../CMakeLists.txt) registers a `KairoPhysicsEngineTests` CTest target. With a qualified C++23 modules toolchain and dependencies:

```bash
cmake -S . -B build -G Ninja -DKAIRO_PHYSICS_ENGINE_BUILD_TESTS=ON
cmake --build build --target KairoPhysicsEngineTests --parallel 4
ctest --test-dir build --output-on-failure -R KairoPhysicsEngineTests
```

Focus future native tests on physically meaningful failure modes:

1. **Anchor rotation:** rotate a body and verify a nonzero local anchor rotates with the body, rather than staying at its old world coordinate.
2. **Distance drift:** compare error over a 10-, 50- and 200-link chain at multiple velocity/position iteration counts, time steps and mass ratios.
3. **Hinge antiparallel axes:** intentionally reverse a hinge axis; compare angle correction against expected π-axis behavior.
4. **Impulse conservation:** with no external forces, track linear momentum and constraint work in two free dynamic bodies under the same joint.
5. **Bilateral sign:** start an anchor pair both shorter and longer than its rest length and verify the joint can push **and** pull.
6. **Static bridges:** repeat the solver-island test with hundreds of separate dynamics sharing one static anchor. Check no unintended union occurs.
7. **Thread validation:** run real serial/parallel parity, sanitizer configurations, independent-island write instrumentation and a stress test on a known host.
8. **Checkpoint continuity:** persist an active articulated scene and compare its next-step simulation to the original, noting the absence of per-joint persistent impulses.

A teaching comment should say **which exact source or assertion supports a claim**, and label what still requires a new experiment. Avoid interpreting one stable joint demo as a guarantee of physically accurate articulation or a real-time performance benchmark.

## 12. Continue into the renderer only after understanding ownership

Once you can trace `JointID` → `SolverIsland` → body pose, move to [KairoRenderer's lifecycle walkthrough](https://github.com/swayam8624/KairoRenderer/blob/main/docs/FRAME_LIFECYCLE_WALKTHROUGH.md). The renderer only consumes derived transforms and debug geometry; it does not own physical identities, constraints or authoritative velocities.

The next major ownership analogy is: a **PhysicsWorld** owns simulation state while its broadphase owns *derived spatial caches*; a **Renderer** owns API-specific GPU resources while its render graph owns a *logical schedule and transient alias plan*. Confusing cache plans with actual ownership or synchronization causes subtle bugs in both subsystems.
