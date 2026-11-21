// LEARNING MAP — public C++23 umbrella module for rigid body simulation.
// `import Kairo.Foundation.PhysicsEngine;` exposes Bodies, Colliders,
// Broadphase, Narrowphase, Solver, Joint, World, serialization and debug APIs.
// This is a package of distinct modules, not a single simulation function.
// See READING_ORDER.md to trace a full simulation step.

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
