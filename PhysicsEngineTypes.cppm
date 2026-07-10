module;

#include <cmath>
#include <stdexcept>
#include <string>

export module Kairo.Foundation.PhysicsEngine.Types;

import Kairo.Foundation.PhysicsMath;

export namespace kairo::foundation::physics
{
    struct PhysicsStepSettings final
    {
        float GravityScale = 1.0f;
        std::uint32_t VelocityIterations = 12;
        float Baumgarte = 0.2f;
        float Slop = 0.01f;
        float MaxPositionCorrection = 0.25f;
    };

    /// Input: step settings and timestep.
    /// Output: throws when simulation parameters are invalid.
    /// Task: keep `PhysicsWorld::Step` deterministic by rejecting invalid dt or
    /// solver controls before mutating body state.
    inline void ValidateStepSettings(
        const PhysicsStepSettings& settings,
        float dt)
    {
        RequirePositive(dt, "dt");
        RequireFinite(settings.GravityScale, "GravityScale");
        RequireNonNegative(settings.Baumgarte, "Baumgarte");
        RequireNonNegative(settings.Slop, "Slop");
        RequireNonNegative(settings.MaxPositionCorrection, "MaxPositionCorrection");

        if (settings.VelocityIterations == 0)
        {
            throw std::invalid_argument("VelocityIterations must be greater than zero.");
        }
    }
}
