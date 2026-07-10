module;

#include <vector>

export module Kairo.Foundation.PhysicsSandbox.PhysicsDebugRenderer;

import Kairo.Foundation.PhysicsEngine;

export namespace kairo::foundation::physics::sandbox
{
    /// Renderer-independent physics debug payload.
    ///
    /// Input: current `PhysicsWorld`.
    /// Output: copied debug contacts and AABBs.
    /// Task: let terminal, GLFW, and future ray-traced export paths consume one
    /// stable debug representation without depending on OpenGL, ImGui, or files.
    struct SandboxDebugSnapshot final
    {
        std::vector<DebugAABB> AABBs;
        std::vector<DebugContact> Contacts;
        std::size_t BroadphasePairs = 0;
    };

    /// Input: world after a simulation step.
    /// Output: debug snapshot copied out of the engine.
    /// Task: isolate visualization code from engine ownership and internal
    /// storage lifetime.
    [[nodiscard]]
    SandboxDebugSnapshot CaptureDebugSnapshot(
        const PhysicsWorld& world)
    {
        return
        {
            world.DebugAABBs(),
            world.DebugContacts(),
            world.BroadphasePairs().size()
        };
    }
}
