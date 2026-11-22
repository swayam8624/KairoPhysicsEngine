# Contact solving, from Newton's laws to the C++23 implementation

Work through this guide with [PhysicsWorld.cppm](../PhysicsWorld.cppm), [Broadphase.cppm](../Broadphase.cppm), [Narrowphase.cppm](../Narrowphase.cppm), [ContactSolver.cppm](../ContactSolver.cppm) and [SolverIsland.cppm](../SolverIsland.cppm) open. The algebraic helpers come from [KairoPhysicsMath](https://github.com/swayam8624/KairoPhysicsMath).

## 1. Identify what each layer owns

**PhysicsWorld** owns rigid bodies, colliders, joints, cached impulses and event keys. **BroadphaseWorld** owns spatial proxies, not shapes; its possible overlaps are not contacts. **Narrowphase** produces contact manifolds by shape testing. **ContactSolver** changes velocities by impulses, while **SolverIsland** groups writable dynamic bodies so independent systems can be solved concurrently.

One body can own many colliders with distinct materials and trigger/filter flags. A BodyID is not a ColliderID, and neither is the dynamic tree's SpatialIndex proxy. A contact normal points from body A toward body B. Reversing A and B also reverses the normal, otherwise impulse signs are wrong.

## 2. From AABB candidates to real contacts

For N objects, naive all-pairs collision testing considers roughly N(N−1)/2 pairs. The dynamic AABB tree prunes distant finite shapes; infinite planes live in a separate list. Broadphase returns canonical, deduplicated candidate pairs, then **ComputeContacts** dispatches shape tests.

The box/box separating-axis test uses 3 face normals from each box plus 9 edge-cross-product axes. If any projected intervals do not overlap, the boxes are separated. A positive test proves geometric overlap for ideal oriented boxes, not the best manifold: **BuildBoxBoxContacts** constructs witnesses and **ReduceContactPoints** retains a limited set useful to the solver. Mesh collisions instead use BVH candidate triangles approximated by thin convex prisms, and are not guaranteed to produce exhaustive mesh/mesh manifolds.

**Exercise:** trace a sphere colliding with an infinite plane. Explain why the plane is not a finite tree leaf, where its contact normal comes from, and why a Trigger response produces an event without a collision impulse.

## 3. Derive the normal impulse

At contact point \(p\), with centers \(x_A,x_B\), define lever arms and point velocities:

\[
r_A=p-x_A,\quad r_B=p-x_B,
\qquad v_{A,p}=v_A+\omega_A\times r_A,\quad v_{B,p}=v_B+\omega_B\times r_B.
\]

For a unit normal \(n\) from A to B, the normal approach speed is

\[
v_n=(v_{B,p}-v_{A,p})\cdot n.
\]

Negative \(v_n\) means bodies approach. An impulse \(jn\) applies \(-jn\) to A and \(+jn\) to B. The scalar effective-mass denominator is

\[
K_n=m_A^{-1}+m_B^{-1}
+(r_A\times n)^T I_A^{-1}(r_A\times n)
+(r_B\times n)^T I_B^{-1}(r_B\times n).
\]

The requested impulse magnitude is the desired relative normal velocity change divided by \(K_n\), with restitution and penetration-bias policy from the actual implementation. A static/kinematic endpoint has zero solver-writable inverse mass. The rotational terms matter for off-center contacts: the same impulse can spin a body.

**BaumgarteBias** adds a velocity correction for penetration beyond slop, scaled by timestep and limited by a correction cap. This is an error-stabilization approximation, not continuous collision detection.

## 4. Calculate one contact manually

Consider center-line (no rotation) collision with normal +x:

| Quantity | A | B |
| --- | ---: | ---: |
| Mass | 2 kg | 3 kg |
| Initial x velocity | +2 m/s | −1 m/s |

With restitution zero and no penetration bias, relative speed is \(v_n=-3\) m/s and \(K_n=1/2+1/3=5/6\) per kilogram. The normal impulse is \(j=3/(5/6)=3.6\) N·s.

After the impulses, \(v_A'=2-3.6/2=0.2\) m/s and \(v_B'=-1+3.6/3=0.2\) m/s. The new relative normal speed is zero. With idealized restitution 0.5, the requested impulse becomes 5.4 N·s and the two velocities become −0.7 and +0.8 m/s, respectively.

**Exercise:** implement the scalar calculation in a tiny standalone C++23 test. Assert conservation of total linear momentum and the expected relative speed. Then use a noncentered contact to test angular response against **ContactNormalDenominator** and **ApplyImpulseAtPoint**.

## 5. Why warm starting and clamped accumulation exist

Contacts are unilateral: their normal impulse must not pull bodies together. The solver clamps **accumulated** normal impulse to the nonnegative range, applies only the increment, and stores the new total in each ContactPoint. In iterative sequential impulses, this prevents applying the same full impulse on each iteration.

**StoreContactCache** records nontrivial normal/tangential magnitudes from blocking contacts. Next step, **RestoreContactCache** matches pair identities, body-local contact anchors and normals before **WarmStartContacts** reapplies those previous multipliers. Local anchors remain meaningful as bodies translate; raw world-space positions would not be stable contact identities.

The cache is simulation state, not authored scene state. PhysicsWorld's versioned snapshot includes cached impulses to support comparable replay, but Editor scene serialization should not save native solver internals by default.

## 6. Understand the friction limitations precisely

Tangential relative velocity is

\[
v_t=v_{\mathrm{rel}}-(v_{\mathrm{rel}}\cdot n)n.
\]

This implementation normalizes nonzero \(v_t\) to one sliding direction \(t\), requests a scalar tangent impulse, and clamps it to a dynamic friction budget \(|j_t|\leq\mu_d j_n\). It **reuses the normal effective-mass denominator** instead of calculating a separate tangential one, and does not persist a two-tangent basis. These are model approximations.

More subtly, **SolveContactManifold** skips tangential work when the newly computed *normal impulse increment* is smaller than its threshold, even when the previously accumulated normal impulse is positive. Thus resting friction may behave differently from a two-direction friction solver. Document and test this behavior rather than assuming textbook Coulomb friction was fully implemented.

**Exercise:** compare inclined-plane slip at several angles with a reference that computes a tangential effective mass. Keep dt, friction coefficient, geometry, initial velocities and solver iteration counts identical.

## 7. Why islands permit safe parallel solving

The solver graph has dynamic bodies as writable nodes and blocking contacts/joints as edges. A disjoint-set structure joins dynamic/dynamic edges into connected components. Static or kinematic anchors can be shared between independent dynamic islands because the solver does not write their velocities.

Each SolverIsland stores **indices** into the current contact/joint arrays. SolveIslandVelocity runs contacts and joints in order for each velocity iteration, and positional correction is a separate pass. Worker threads may process different islands, assuming no shared dynamic body/constraint is mutable from two islands.

**Exercise:** create A(dynamic)–Ground(static)–B(dynamic). Explain why the dynamic bodies can remain in separate islands. Then add a direct joint A–B and verify that union-find merges them.

## 8. Follow one actual PhysicsWorld step

**Step(dt)** validates settings and rejects recursive calls; integrates dynamic forces and moves kinematic bodies; syncs spatial proxies; computes candidate pairs and shape contacts; filters Ignore/Trigger/Block responses; prepares Begin/Stay/End events; wakes bodies and builds islands; restores cached multipliers; solves velocities and stores cache; corrects penetration; advances dynamic bodies with bounded CCD; updates sleeping; clears one-step forces; records CPU timings; and invokes a copied callback/event list only after state is finalized.

**StepFixed(elapsed,fixedDt,maxSubSteps)** repeatedly calls Step with a constant dt, but discards excess backlog beyond the configured cap. Fixed-step integration improves consistency; it is not a guarantee of cross-platform bitwise determinism or exact catch-up with wall-clock time.

## 9. Testing and evidence

Begin with [PhysicsEngineTests.cpp](../tests/PhysicsEngineTests.cpp) and [PhysicsStressTests.cpp](../tests/PhysicsStressTests.cpp). Add explicit cases for collision normal sign, off-center rotation, resting contact warm start, blocked/trigger response, snapshot restoration, island sequential-vs-parallel outputs and continuous sphere/plane impact. Record numerical tolerances, timestep, iterations, seed, source SHA and exact compiler before claiming numerical parity or timing improvement.

Candidate pair count is not actual contact count. A smaller candidate set is not proof of better physical correctness; performance and fidelity need separate controlled evidence.

*Status: this is a code-reading lesson. No new runtime build or benchmark is claimed.*
