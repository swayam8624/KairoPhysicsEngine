module;

#include <cstdint>
#include <string>
#include <vector>

export module Kairo.Foundation.PhysicsSandbox.Types;

import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine;

export namespace kairo::foundation::physics::sandbox
{
    /// Demo scenarios supported by the terminal sandbox.
    ///
    /// Input: selected by command-line name or future scene file metadata.
    /// Output: a deterministic physics setup built by `MakeDemoScene`.
    /// Task: keep terminal, benchmark, window, and future ray-traced export
    /// paths talking about the same canonical scenes instead of each tool
    /// inventing its own test setup.
    enum class SandboxScenario : std::uint8_t
    {
        FallingSphere,
        SphereCollision,
        BoxStack,
        FrictionRamp,
        RestitutionTest,
        SleepingTest,
        Stress100Spheres,
        Stress500Spheres
    };

    /// Runtime controls for deterministic sandbox playback.
    ///
    /// Input: CLI flags or a future `.kphys` scene file.
    /// Output: fixed-step run configuration.
    /// Task: separate simulation settings from demo construction so terminal,
    /// benchmark, and export tools can share one scenario builder.
    struct SandboxRunSettings final
    {
        SandboxScenario Scenario = SandboxScenario::FallingSphere;
        std::uint32_t Steps = 180;
        std::uint32_t PrintEvery = 30;
        float FixedDt = 1.0f / 60.0f;
        bool ShowAscii = true;
        bool PrintBodies = true;
        std::string CsvPath;
    };

    /// Body metadata used by text and debug renderers.
    ///
    /// Input: body/collider handles returned by `PhysicsWorld`.
    /// Output: stable label and glyph for terminal side-view rendering.
    /// Task: keep render/debug metadata out of engine body records.
    struct TrackedBody final
    {
        BodyID Body = InvalidBodyID;
        ColliderID Collider = InvalidColliderID;
        std::string Name;
        char Glyph = 'o';
    };

    /// Full sandbox scene instance.
    ///
    /// Input: scenario construction function.
    /// Output: owned `PhysicsWorld` plus tracked bodies and documentation text.
    /// Task: give the sandbox a clear owner for world state while preserving
    /// engine independence.
    struct SandboxScene final
    {
        PhysicsWorld World;
        std::vector<TrackedBody> TrackedBodies;
        std::string Name;
        std::string Description;
    };

    /// Per-frame terminal and CSV metrics.
    ///
    /// Input: current world state after zero or more fixed steps.
    /// Output: compact values suitable for console logs and benchmark CSV.
    /// Task: make physics behavior visible numerically before any window or
    /// ray-traced export exists.
    struct SandboxFrameStats final
    {
        std::uint32_t Step = 0;
        float TimeSeconds = 0.0f;
        std::size_t Bodies = 0;
        std::size_t Contacts = 0;
        std::size_t Islands = 0;
        std::size_t AwakeBodies = 0;
        std::size_t SleepingBodies = 0;
        double StepMs = 0.0;
        double BroadphaseMs = 0.0;
        double NarrowphaseMs = 0.0;
        double SolverMs = 0.0;
    };
}
