// TEACHING ENTRY POINT: Public C++23 umbrella for the physics *engine*.
// In contrast with KairoPhysicsMath (stateless formulas), this repository owns
// bodies, colliders, spatial proxies, warm-start caches, fixed-step progression,
// event state, joints, scene queries and observation/replay interfaces.
//
// A useful conceptual trace is:
// PhysicsWorld -> Broadphase (possible pairs) -> Narrowphase/ConvexCollision
// (actual contact geometry) -> SolverIsland -> ContactSolver/Joint (impulses)
// -> world state, contact Begin/Stay/End events and debug/replay consumers.
// The exact execution sequence must be verified in PhysicsWorld::Step; the
// export order below is *not* a runtime call sequence.
//
// Physical math is supplied by KairoPhysicsMath; shapes/intersections by
// KairoGeometry; acceleration structures by KairoSpatial. The renderer
// consumes debug data but does not determine simulation outcomes.
// Distinguish discrete contacts from continuous sweeps/TOI and do not mistake
// a broadphase overlap for a collision.
//
// `export module` declares this umbrella; `export import` republishes the
// narrower interfaces to its consumers. See docs/SOURCE_READING_ORDER.md,
// tests/PhysicsEngineTests.cpp and tests/PhysicsStressTests.cpp.

export module Kairo.Foundation.PhysicsEngine;

export import Kairo.Foundation.PhysicsEngine.Types;
export import Kairo.Foundation.PhysicsEngine.Material;
export import Kairo.Foundation.PhysicsEngine.RigidBody;
export import Kairo.Foundation.PhysicsEngine.Collider;
export import Kairo.Foundation.PhysicsEngine.Broadphase;
export import Kairo.Foundation.PhysicsEngine.ConvexCollision;
export import Kairo.Foundation.PhysicsEngine.Narrowphase;
export import Kairo.Foundation.PhysicsEngine.ContactSolver;
export import Kairo.Foundation.PhysicsEngine.Joint;
export import Kairo.Foundation.PhysicsEngine.SolverIsland;
export import Kairo.Foundation.PhysicsEngine.Debug;
export import Kairo.Foundation.PhysicsEngine.World;
export import Kairo.Foundation.PhysicsEngine.Serialization;
export import Kairo.Foundation.PhysicsEngine.Replay;
export import Kairo.Foundation.PhysicsEngine.Projectile;
export import Kairo.Foundation.PhysicsEngine.Buoyancy;
export import Kairo.Foundation.PhysicsEngine.VehicleSystem;
export import Kairo.Foundation.PhysicsEngine.CharacterController;
