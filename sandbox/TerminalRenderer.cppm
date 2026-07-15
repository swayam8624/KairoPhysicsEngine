module;

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <ostream>
#include <string>
#include <vector>

export module Kairo.Foundation.PhysicsSandbox.TerminalRenderer;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.PhysicsEngine;
import Kairo.Foundation.PhysicsSandbox.Types;

export namespace kairo::foundation::physics::sandbox
{
    using namespace kairo::foundation::math;

    namespace detail
    {
        [[nodiscard]]
        std::size_t FindRoot(
            std::vector<std::size_t>& parent,
            std::size_t index)
        {
            while (parent.at(index) != index)
            {
                parent.at(index) = parent.at(parent.at(index));
                index = parent.at(index);
            }

            return index;
        }

        void Union(
            std::vector<std::size_t>& parent,
            std::size_t a,
            std::size_t b)
        {
            const std::size_t rootA =
                FindRoot(parent, a);

            const std::size_t rootB =
                FindRoot(parent, b);

            if (rootA != rootB)
            {
                parent.at(rootB) = rootA;
            }
        }

        [[nodiscard]]
        bool IsRenderableBody(
            const PhysicsWorld& world,
            BodyID body)
        {
            return body < world.Bodies().size() &&
                IsActiveBody(world.Bodies().at(body));
        }
    }

    using namespace detail;

    /// Input: scene, current step, and current simulated time.
    /// Output: aggregate frame metrics for terminal and CSV output.
    /// Task: make the sandbox useful before visualization by reporting the same
    /// values an engine developer watches during simulation debugging.
    [[nodiscard]]
    SandboxFrameStats ComputeFrameStats(
        const SandboxScene& scene,
        std::uint32_t step,
        float timeSeconds)
    {
        const PhysicsWorld& world =
            scene.World;

        SandboxFrameStats stats;
        stats.Step = step;
        stats.TimeSeconds = timeSeconds;
        stats.Bodies =
            std::count_if(
                world.Bodies().begin(),
                world.Bodies().end(),
                [](const RigidBody& body)
                {
                    return IsActiveBody(body);
                });
        stats.Contacts = world.Contacts().size();

        for (const RigidBody& body : world.Bodies())
        {
            if (!IsActiveBody(body) || !IsDynamicBodyType(body))
            {
                continue;
            }

            if (body.Sleeping)
            {
                ++stats.SleepingBodies;
            }
            else
            {
                ++stats.AwakeBodies;
            }
        }

        std::vector<std::size_t> parent(world.Bodies().size());
        std::iota(parent.begin(), parent.end(), std::size_t{ 0 });

        for (const ContactManifold& contact : world.Contacts())
        {
            if (contact.BodyA < parent.size() && contact.BodyB < parent.size())
            {
                Union(parent, contact.BodyA, contact.BodyB);
            }
        }

        std::vector<std::size_t> roots;
        for (const RigidBody& body : world.Bodies())
        {
            if (!IsActiveBody(body) || !IsDynamicBodyType(body))
            {
                continue;
            }

            const std::size_t root =
                FindRoot(parent, body.ID);

            if (std::find(roots.begin(), roots.end(), root) == roots.end())
            {
                roots.push_back(root);
            }
        }

        stats.Islands = roots.size();

        const PhysicsStepProfile& profile =
            world.LastStepProfile();

        stats.StepMs = profile.StepMs;
        stats.BroadphaseMs = profile.BroadphaseMs;
        stats.NarrowphaseMs = profile.NarrowphaseMs;
        stats.SolverMs = profile.SolverMs;
        return stats;
    }

    /// Input: output stream, scene name, and frame stats.
    /// Output: human-readable summary block.
    /// Task: match the requested terminal form so physics behavior is readable
    /// even over SSH or CI logs.
    void PrintFrameSummary(
        std::ostream& out,
        const SandboxScene& scene,
        const SandboxFrameStats& stats)
    {
        out
            << "scene: " << scene.Name << '\n'
            << "time: " << std::fixed << std::setprecision(3) << stats.TimeSeconds << "s\n"
            << "bodies: " << stats.Bodies << '\n'
            << "contacts: " << stats.Contacts << '\n'
            << "islands: " << stats.Islands << '\n'
            << "awake: " << stats.AwakeBodies << '\n'
            << "sleeping: " << stats.SleepingBodies << '\n'
            << "step_ms: " << std::setprecision(4) << stats.StepMs << '\n';
    }

    /// Input: output stream and tracked bodies.
    /// Output: one body row per active tracked body.
    /// Task: expose positions and velocities exactly enough for numerical
    /// debugging without dumping the entire engine record.
    void PrintBodyRows(
        std::ostream& out,
        const SandboxScene& scene)
    {
        for (const TrackedBody& tracked : scene.TrackedBodies)
        {
            if (!IsRenderableBody(scene.World, tracked.Body))
            {
                continue;
            }

            const RigidBody& body =
                scene.World.Bodies().at(tracked.Body);

            out
                << tracked.Name
                << " body[" << tracked.Body << "]"
                << " pos=("
                << std::fixed << std::setprecision(2)
                << body.State.Position.x << ", "
                << body.State.Position.y << ", "
                << body.State.Position.z << ")"
                << " vel=("
                << body.State.LinearVelocity.x << ", "
                << body.State.LinearVelocity.y << ", "
                << body.State.LinearVelocity.z << ")"
                << (body.Sleeping ? " sleeping" : " awake")
                << '\n';
        }
    }

    /// Input: scene and output size in characters.
    /// Output: ASCII side-view written to stream.
    /// Task: provide a cheap visual debugger for terminal-only workflows.
    void PrintAsciiSideView(
        std::ostream& out,
        const SandboxScene& scene,
        int width = 48,
        int height = 14)
    {
        const float minX = -6.0f;
        const float maxX = 6.0f;
        const float minY = 0.0f;
        const float maxY = 7.0f;

        std::vector<std::string> grid(
            static_cast<std::size_t>(height),
            std::string(static_cast<std::size_t>(width), ' '));

        for (const TrackedBody& tracked : scene.TrackedBodies)
        {
            if (!IsRenderableBody(scene.World, tracked.Body))
            {
                continue;
            }

            const Vec3f position =
                scene.World.Bodies().at(tracked.Body).State.Position;

            const int x =
                static_cast<int>(
                    std::lround(
                        (std::clamp(position.x, minX, maxX) - minX) /
                        (maxX - minX) *
                        static_cast<float>(width - 1)));

            const int y =
                static_cast<int>(
                    std::lround(
                        (std::clamp(position.y, minY, maxY) - minY) /
                        (maxY - minY) *
                        static_cast<float>(height - 1)));

            const int row =
                std::clamp(height - 1 - y, 0, height - 1);

            const int column =
                std::clamp(x, 0, width - 1);

            grid.at(static_cast<std::size_t>(row)).at(static_cast<std::size_t>(column)) =
                tracked.Glyph;
        }

        out << "y\n|\n";
        for (const std::string& row : grid)
        {
            out << "| " << row << '\n';
        }
        out << "+" << std::string(static_cast<std::size_t>(width + 1), '_') << " floor\n";
    }

    /// Input: writable stream.
    /// Output: CSV header for benchmark/stress output.
    /// Task: standardize performance captures so future optimization passes can
    /// compare broadphase, narrowphase, and solver changes.
    void WriteCsvHeader(
        std::ostream& out)
    {
        out
            << "frame,time,bodies,contacts,islands,awake,sleeping,"
            << "step_ms,broadphase_ms,narrowphase_ms,solver_ms\n";
    }

    /// Input: writable stream and frame stats.
    /// Output: one CSV record.
    /// Task: persist benchmark data without post-processing the console text.
    void WriteCsvRow(
        std::ostream& out,
        const SandboxFrameStats& stats)
    {
        out
            << stats.Step << ','
            << std::fixed << std::setprecision(6)
            << stats.TimeSeconds << ','
            << stats.Bodies << ','
            << stats.Contacts << ','
            << stats.Islands << ','
            << stats.AwakeBodies << ','
            << stats.SleepingBodies << ','
            << stats.StepMs << ','
            << stats.BroadphaseMs << ','
            << stats.NarrowphaseMs << ','
            << stats.SolverMs << '\n';
    }
}
