module;

#include <algorithm>
#include <cmath>

export module Kairo.Foundation.PhysicsEngine.Material;

import Kairo.Foundation.PhysicsMath;

export namespace kairo::foundation::physics
{
    struct PhysicsMaterial final
    {
        float Restitution = 0.1f;
        float StaticFriction = 0.6f;
        float DynamicFriction = 0.45f;
    };

    /// Input: material.
    /// Output: throws when coefficients are invalid.
    /// Task: enforce non-negative material coefficients at collider boundaries.
    inline void ValidatePhysicsMaterial(
        const PhysicsMaterial& material)
    {
        RequireNonNegative(material.Restitution, "Restitution");
        RequireNonNegative(material.StaticFriction, "StaticFriction");
        RequireNonNegative(material.DynamicFriction, "DynamicFriction");
    }

    /// Input: two contact materials.
    /// Output: mixed restitution using conservative minimum.
    /// Task: avoid surprise energy gain when one material is intentionally dull.
    [[nodiscard]]
    inline float MixRestitution(
        const PhysicsMaterial& a,
        const PhysicsMaterial& b)
    {
        return std::min(a.Restitution, b.Restitution);
    }

    /// Input: two contact materials.
    /// Output: geometric mean friction.
    /// Task: standard stable friction mixing for pairwise contacts.
    [[nodiscard]]
    inline float MixFriction(
        float a,
        float b)
    {
        return std::sqrt(std::max(0.0f, a) * std::max(0.0f, b));
    }
}
