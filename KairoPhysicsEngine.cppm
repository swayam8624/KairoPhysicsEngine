// SOURCE STUDY NOTE:
// C++23 umbrella module re-exporting physics types, collision stages, world, debug, replay and
// serialization. Consumers such as KairoPlayer may import the umbrella or specific modules; source
// implementations remain divided across .cppm files.
// See docs/SOURCE_READING_ORDER.md for upstream/downstream APIs and tests.

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
