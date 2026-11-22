# PhysicsWorld laboratory: stable body IDs, contact lifetime, warm starts and CCD

Read [PhysicsWorld.cppm](../PhysicsWorld.cppm), [Broadphase.cppm](../Broadphase.cppm), [Narrowphase.cppm](../Narrowphase.cppm), [ContactSolver.cppm](../ContactSolver.cppm) and [SolverIsland.cppm](../SolverIsland.cppm). The [collision pipeline walkthrough](COLLISION_PIPELINE_WALKTHROUGH.md) and [worked impulse equations](CONTACT_SOLVER_FROM_EQUATIONS_TO_CODE.md) provide the wider derivation. This lesson follows one moving body through ownership, stepping, event delivery, snapshots and continuous-impact limitations.

## 1. Distinguish stable identities from transient spatial handles

PhysicsWorld::CreateRigidBody appends one RigidBody to m_Bodies and returns its vector index as BodyID. AddCollider allocates a separate ColliderID referencing that body; one body may carry more than one collider. A deleted body or collider is marked inactive without compacting IDs, so old index values cannot alias a newly inserted object inside the same world.

The spatial broad phase owns a different, **transient** DynamicAABBTree SpatialIndex for finite collider proxies, plus a separate list for infinite planes. Clear/rebuild or category changes can replace a tree handle without changing an authoritative ColliderID. The Player adapter maintains yet another EngineCore authored Entity -> BodyID mapping; none of these process-local numbers should be serialized as an authored stable Entity ID.

**Exercise:** create one dynamic body with two colliders, remove the first, and create another collider. Trace its BodyID, old inactive ColliderID, new ColliderID, and spatial proxy handle. Explain why no test may infer equality from identical numbers in these separate namespaces.

## 2. What one PhysicsWorld::Step actually does

The key call sequence is:

1. Reject recursive stepping, validate dt, gravity and PhysicsStepSettings.
2. Integrate force/torque into **velocities**, and advance kinematic motion.
3. Synchronize current shape bounds into BroadphaseWorld and obtain possible collider pairs.
4. Narrowphase computes actual shape contacts; PhysicsWorld resolves Ignore, Trigger or Block policy.
5. Generate pending Begin/Stay/End contact transitions, wake connected bodies and construct solver islands.
6. Restore cached manifold impulses, warm-start contacts, iteratively solve contacts and joints, then store impulses and correct residual positions.
7. Advance dynamic positions, using bounded continuous-impact handling where enabled; update sleeping, clear accumulated forces and publish per-stage profiling.
8. Dispatch pending contact events only **after** completed world mutation.

The RAII StepGuard resets the recursive-step flag even when validation or callback processing throws. It is a reentrancy guard, **not** a general rollback transaction: an exception later in stepping may still leave some mutated simulation data. A saved snapshot or transactional host must provide recovery if that failure mode matters.

**Exercise:** instrument Stage 3's candidate count, Stage 4's true manifold count and Stage 8's event count for a falling sphere. Show why those numbers need not match.

## 3. A candidate AABB pair is not a contact

The dynamic tree accelerates broad-phase queries using fat AABB proxies. Finite boxes that overlap can belong to separated or rotated shapes; two spheres' AABBs can overlap while the spheres remain apart. Infinite planes cannot be stored in a finite conservative AABB, so BroadphaseWorld tests their collider IDs separately.

ShouldPair checks active shape and parent body IDs, distinct parent bodies and mutual collision-category filters. PhysicsWorld's later collision response policy may still Ignore, Trigger or Block a geometrically valid manifold. A trigger participates in contact reporting but is explicitly skipped by ContactSolver's impulse path.

**Exercise:** use two overlapping sensor volumes and verify events can occur without separating impulses. Switch the same geometry to Block and compare the resulting velocities.

## 4. Derive a contact normal impulse

Let n point from body A toward body B. At a world contact point p, the lever arms are rA=p−xA and rB=p−xB. Contact relative velocity includes angular movement:

\[
v_\mathrm{rel}=(v_B+\omega_B\times r_B)-(v_A+\omega_A\times r_A).
\]

The normal denominator contains inverse masses and rotational leverage from the world inverse inertia tensors. The requested normal impulse changes relative normal velocity while respecting restitution and penetration stabilization.

For a simple centered, frictionless, one-dimensional case with equal mass 2 kg, body A approaching at +3 m/s and B stationary, with restitution e=0, effective inverse mass is 1/2+1/2=1 kg^{-1}. The closing normal speed is −3 m/s under the appropriate ordering, so the impulse magnitude is 3 N·s; velocities become +1.5 and +1.5 m/s. For e=1, magnitude 6 N·s and velocities exchange, A=0 and B=3.

ContactSolver projects the **accumulated** normal impulse to be nonnegative, then applies only the difference from the previous accumulator. That enforces unilateral contact: bodies may push but not pull one another together. Contact points store normal and tangent accumulators to warm-start the next step.

The implementation has a specific approximation: a very small change in normal impulse skips that contact point's tangential solve during the iteration. The tangent accumulator is also a scalar reconstructed against current sliding velocity rather than a persistent two-dimensional basis. These details can affect resting friction and should be tested instead of claiming a fully general Coulomb solver.

## 5. Begin/Stay/End and callback mutation

PhysicsWorld::UpdateContactEvents canonicalizes the current manifold keys and compares them with the prior step's keys. Newly present keys produce Begin, continuing keys Stay, and missing previous keys End. These transitions are prepared before solving; DispatchContactEvents is deliberately deferred to Step's end.

The dispatcher copies the callback and event vector before calling gameplay. This allows a callback to unsubscribe, create or remove an entity or adjust world topology without invalidating its current event iteration. Calling Step recursively remains prohibited.

**Exercise:** have a sphere enter, remain inside and exit a trigger volume. Record Begin, Stay, End and check that trigger events never inject normal impulses. Then let the callback remove a body and ensure the current event dispatch operates over the copied event collection.

## 6. Continuous impact is a different calculation

A discrete solver tests contact at sampled states. A rapidly moving sphere may pass a thin barrier between two samples. PhysicsWorld supports an opt-in CollisionDetectionMode::Continuous path which sweeps supported collider approximations along proposed displacement and clamps movement near an estimated time of impact.

ApplyContinuousImpactImpulse resolves approaching relative linear speed using mass and restitution. This is bounded early-impact handling rather than a general proof of intersection-free motion for arbitrary moving convex/mesh shapes. Angular effects, unsupported source geometry, iteration tolerance and numerical approximation can all limit coverage.

**Worked example:** sphere center at x=0, radius 0.1 m, velocity 100 m/s, dt=1/60 s. One naive step translates by about 1.667 m. A wall only 0.1 m thick could be crossed without a discrete overlap at either endpoint. A continuous sweep must test the swept path, not just enlarged endpoint boxes.

**Exercise:** use the projectile-wall sandbox fixture with discrete versus continuous detection, vary speed and dt, and record candidate/TOI results and residual penetration. Do not claim CCD guarantees based solely on a successful visible demo.

## 7. Fixed stepping and host frame time are not identical

PhysicsWorld::Step(dt) accepts one fixed simulation interval. StepFixed(elapsedTime,fixedDt,maxSubSteps) accumulates host time and executes integer Step calls up to its substep cap. When capped with a full step still queued, this implementation sets the remaining accumulator to **zero**. It sacrifices some elapsed time to maintain bounded work.

KairoPlayer's separate RuntimePhysicsBridge adapter has its own clamped frame delta, capped substeps and interpolation policy. Its substep overload retains only a fractional remainder with fmod. Compare these two concrete implementations rather than assuming identical backlog behavior across layers.

**Exercise:** take fixedDt=0.02 s, maxSubSteps=3 and elapsedTime=0.10 s. PhysicsWorld::StepFixed runs at most three steps and discards excess backlog. Compute its simulated advance (0.06 s) versus input (0.10 s). Then trace Player adapter with the same constraints and identify where the fractional remainder differs.

## 8. Why a snapshot includes more than transforms

PhysicsWorldSnapshot captures bodies, colliders, joints, settings, gravity, last pairs/manifolds, response policy, fixed accumulator, previous event keys, and cached normal/tangent impulses. A replay with only body positions would change future solver warm starts and contact event continuity.

RestoreSnapshot validates topology and rebuilds derived collider/broad-phase structures. Native callback closures and wall-clock profiling are excluded from the saved state because they are process bindings rather than deterministic solver state.

The Player's authored Entity -> BodyID map is **outside PhysicsWorld** and must be restored/validated consistently with it after streaming or save-game load. The bridge performs explicit topology checks before accepting a saved physics state.

**Exercise:** save a resting box stack after contact warm starts. Restore and compare the next-step contact keys and body motion with an uninterrupted control run. Repeat after deleting one body and explain why a mismatched external Entity map cannot be silently reused.

## 9. Test plan and limitations

Start with [PhysicsEngineTests.cpp](../tests/PhysicsEngineTests.cpp), [PhysicsStressTests.cpp](../tests/PhysicsStressTests.cpp), [CharacterControllerTests.cpp](../tests/CharacterControllerTests.cpp) and the scene adapters in [KairoGameEngine's RuntimePhysicsTopologyTests.cpp](https://github.com/swayam8624/KairoGameEngine/blob/main/Runtime/KairoPlayer/tests/RuntimePhysicsTopologyTests.cpp).

Verify inactive identity, multiple colliders, finite versus infinite broadphase, trigger/no-impulse, contact event mutation, friction slide stability, zero denominator, warm-start benefit, snapshot replay, recursive Step rejection and high-speed swept motion. Record source SHA, toolchain, dt, solver iteration counts, active body count, actual fixture, full-step/profile timings and a numerical oracle.

Passing tests on a finite suite does not prove all future geometry, CCD or solver correctness. The physics UI is a diagnostic consumer, not the authority for simulated state.

*Source-derived teaching documentation. No new C++ compilation, physics sandbox run or native GPU verification was performed for this lab.*
