#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_NONE
#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <GLFW/glfw3.h>

import Kairo.Foundation.PhysicsEngine;
import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.Math.Vector;

using namespace kairo::foundation::math;
using namespace kairo::foundation::physics;

namespace
{
    constexpr float Pi =
        3.14159265358979323846f;

    constexpr float ViewLeft =
        -9.5f;

    constexpr float ViewRight =
        9.5f;

    constexpr float ViewBottom =
        -1.2f;

    constexpr float ViewTop =
        8.8f;

    constexpr float PlayLeft =
        -8.35f;

    constexpr float PlayRight =
        8.35f;

    constexpr float PlayBottom =
        0.0f;

    constexpr float PlayTop =
        7.8f;

    constexpr float PushAcceleration =
        14.0f;

    constexpr float DirectMoveSpeed =
        2.15f;

    constexpr float TorqueAcceleration =
        3.2f;

    constexpr float RayMaxDistance =
        14.0f;

    constexpr int RayFanCount =
        17;

    constexpr float RayFanSpread =
        1.35f;

    constexpr float RayRotationSpeed =
        1.8f;

    constexpr float CollisionDemoSpeed =
        4.4f;

    enum class ActorKind
    {
        Sphere,
        Box,
        StaticBox,
        Trigger
    };

    struct Color final
    {
        float R = 1.0f;
        float G = 1.0f;
        float B = 1.0f;
        float A = 1.0f;
    };

    struct Actor final
    {
        BodyID Body = InvalidBodyID;
        ColliderID Collider = InvalidColliderID;
        ActorKind Kind = ActorKind::Sphere;
        float Radius = 0.5f;
        Vec3f HalfExtents = Vec3f{ 0.5f, 0.5f, 0.5f };
        Quaternionf LocalRotation = Quaternionf::Identity();
        Color Fill;
        bool Selectable = true;
    };

    struct KeyLatch final
    {
        bool Previous = false;

        [[nodiscard]]
        bool Pressed(
            bool down)
        {
            const bool pressed =
                down && !Previous;

            Previous = down;
            return pressed;
        }
    };

    struct SandboxState final
    {
        PhysicsWorld World;
        std::vector<Actor> Actors;
        std::size_t Selected = 0;
        bool Paused = false;
        bool ShowDebug = true;
        bool GravityEnabled = true;
        bool ShowRays = true;
        float RayAngle = 0.0f;
        std::uint32_t CallbackEvents = 0;
        std::uint32_t TriggerCallbackEvents = 0;
        int SpawnIndex = 0;
    };

    [[nodiscard]]
    PhysicsMaterial BouncyMaterial()
    {
        PhysicsMaterial material;
        material.Restitution = 0.35f;
        material.StaticFriction = 0.65f;
        material.DynamicFriction = 0.45f;
        return material;
    }

    [[nodiscard]]
    PhysicsMaterial GroundMaterial()
    {
        PhysicsMaterial material;
        material.Restitution = 0.05f;
        material.StaticFriction = 0.9f;
        material.DynamicFriction = 0.7f;
        return material;
    }

    [[nodiscard]]
    RigidBodyDesc DynamicSphereBody(
        const Vec3f& position,
        float radius)
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Dynamic;
        desc.State.Position = position;
        desc.Mass = SphereMassProperties(radius, 1.0f);
        desc.LinearDamping = 0.22f;
        desc.AngularDamping = 0.45f;
        desc.MaxLinearSpeed = 11.0f;
        desc.MaxAngularSpeed = 12.0f;
        desc.AllowSleeping = true;
        return desc;
    }

    [[nodiscard]]
    RigidBodyDesc DynamicBoxBody(
        const Vec3f& position,
        const Vec3f& halfExtents)
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Dynamic;
        desc.State.Position = position;
        desc.State.Rotation = RotationAroundZ(0.15f);
        desc.Mass = BoxMassProperties(halfExtents, 1.0f);
        desc.LinearDamping = 0.22f;
        desc.AngularDamping = 0.45f;
        desc.MaxLinearSpeed = 11.0f;
        desc.MaxAngularSpeed = 12.0f;
        desc.AllowSleeping = true;
        return desc;
    }

    [[nodiscard]]
    RigidBodyDesc StaticBoxBody(
        const Vec3f& position,
        float radians = 0.0f)
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Static;
        desc.State.Position = position;
        desc.State.Rotation = RotationAroundZ(radians);
        return desc;
    }

    [[nodiscard]]
    bool ActorAlive(
        const SandboxState& state,
        const Actor& actor)
    {
        return state.World.IsValidBody(actor.Body) &&
            state.World.IsValidCollider(actor.Collider);
    }

    [[nodiscard]]
    Actor* SelectedActor(
        SandboxState& state)
    {
        if (state.Actors.empty())
        {
            return nullptr;
        }

        for (std::size_t attempt = 0; attempt < state.Actors.size(); ++attempt)
        {
            state.Selected %= state.Actors.size();
            Actor& actor =
                state.Actors.at(state.Selected);

            if (actor.Selectable && ActorAlive(state, actor))
            {
                return &actor;
            }

            state.Selected =
                (state.Selected + 1u) % state.Actors.size();
        }

        return nullptr;
    }

    void SelectNext(
        SandboxState& state,
        int direction)
    {
        if (state.Actors.empty())
        {
            return;
        }

        for (std::size_t attempt = 0; attempt < state.Actors.size(); ++attempt)
        {
            if (direction >= 0)
            {
                state.Selected =
                    (state.Selected + 1u) % state.Actors.size();
            }
            else
            {
                state.Selected =
                    state.Selected == 0 ? state.Actors.size() - 1u : state.Selected - 1u;
            }

            Actor& actor =
                state.Actors.at(state.Selected);

            if (actor.Selectable && ActorAlive(state, actor))
            {
                return;
            }
        }
    }

    Actor& AddSphere(
        SandboxState& state,
        const Vec3f& position,
        float radius,
        const Color& color)
    {
        const BodyID body =
            state.World.CreateRigidBody(DynamicSphereBody(position, radius));

        const ColliderID collider =
            state.World.AddCollider(body, SphereCollider{ radius }, BouncyMaterial());

        state.World.SetCollisionFilter(
            collider,
            CollisionLayer::DynamicWorld,
            CollisionLayer::All);

        state.Actors.push_back(
            Actor
            {
                body,
                collider,
                ActorKind::Sphere,
                radius,
                Vec3f{ radius, radius, radius },
                Quaternionf::Identity(),
                color,
                true
            });

        state.Selected =
            state.Actors.size() - 1u;

        return state.Actors.back();
    }

    Actor& AddBox(
        SandboxState& state,
        const Vec3f& position,
        const Vec3f& halfExtents,
        const Color& color)
    {
        const BodyID body =
            state.World.CreateRigidBody(DynamicBoxBody(position, halfExtents));

        const ColliderID collider =
            state.World.AddCollider(body, BoxCollider{ halfExtents }, BouncyMaterial());

        state.World.SetCollisionFilter(
            collider,
            CollisionLayer::DynamicWorld,
            CollisionLayer::All);

        state.Actors.push_back(
            Actor
            {
                body,
                collider,
                ActorKind::Box,
                0.0f,
                halfExtents,
                Quaternionf::Identity(),
                color,
                true
            });

        state.Selected =
            state.Actors.size() - 1u;

        return state.Actors.back();
    }

    Actor& AddStaticBox(
        SandboxState& state,
        const Vec3f& position,
        const Vec3f& halfExtents,
        float radians,
        const Color& color)
    {
        const BodyID body =
            state.World.CreateRigidBody(StaticBoxBody(position, radians));

        const ColliderID collider =
            state.World.AddCollider(body, BoxCollider{ halfExtents }, GroundMaterial());

        state.World.SetCollisionFilter(
            collider,
            CollisionLayer::StaticWorld,
            CollisionLayer::All);

        state.Actors.push_back(
            Actor
            {
                body,
                collider,
                ActorKind::StaticBox,
                0.0f,
                halfExtents,
                Quaternionf::Identity(),
                color,
                false
            });

        return state.Actors.back();
    }

    Actor& AddTrigger(
        SandboxState& state,
        const Vec3f& position,
        const Vec3f& halfExtents,
        const Color& color)
    {
        const BodyID body =
            state.World.CreateRigidBody(StaticBoxBody(position));

        const ColliderID collider =
            state.World.AddCollider(body, BoxCollider{ halfExtents }, GroundMaterial());

        state.World.SetCollisionFilter(
            collider,
            CollisionLayer::Trigger,
            CollisionLayer::All);

        state.World.SetColliderTrigger(collider, true);

        state.Actors.push_back(
            Actor
            {
                body,
                collider,
                ActorKind::Trigger,
                0.0f,
                halfExtents,
                Quaternionf::Identity(),
                color,
                false
            });

        return state.Actors.back();
    }

    void ResetWorld(
        SandboxState& state)
    {
        const bool paused =
            state.Paused;

        const bool showDebug =
            state.ShowDebug;

        const bool gravityEnabled =
            state.GravityEnabled;

        const bool showRays =
            state.ShowRays;

        const float rayAngle =
            state.RayAngle;

        state = SandboxState{};
        state.Paused = paused;
        state.ShowDebug = showDebug;
        state.GravityEnabled = gravityEnabled;
        state.ShowRays = showRays;
        state.RayAngle = rayAngle;
        state.World.Settings.EnableSleeping = false;
        state.World.Settings.VelocityIterations = 18;
        state.World.Settings.PositionIterations = 8;
        state.World.Settings.SleepTime = 1.0f;
        state.World.Gravity = state.GravityEnabled ? DefaultGravity : Vec3f::Zero();
        state.World.SetCollisionLayerResponse(
            CollisionLayer::DynamicWorld,
            CollisionLayer::StaticWorld,
            CollisionResponse::Block);
        state.World.SetCollisionLayerResponse(
            CollisionLayer::DynamicWorld,
            CollisionLayer::DynamicWorld,
            CollisionResponse::Block);
        state.World.SetCollisionLayerResponse(
            CollisionLayer::DynamicWorld,
            CollisionLayer::Trigger,
            CollisionResponse::Trigger);
        state.World.SetContactEventCallback(
            [&state](const PhysicsContactEvent& event)
            {
                ++state.CallbackEvents;
                if (event.Response == CollisionResponse::Trigger)
                {
                    ++state.TriggerCallbackEvents;
                }
            });

        AddStaticBox(state, Vec3f{ 0.0f, -0.25f, 0.0f }, Vec3f{ 8.65f, 0.25f, 0.5f }, 0.0f, Color{ 0.34f, 0.37f, 0.42f, 1.0f });
        AddStaticBox(state, Vec3f{ -8.65f, 3.85f, 0.0f }, Vec3f{ 0.30f, 4.35f, 0.5f }, 0.0f, Color{ 0.30f, 0.32f, 0.36f, 1.0f });
        AddStaticBox(state, Vec3f{ 8.65f, 3.85f, 0.0f }, Vec3f{ 0.30f, 4.35f, 0.5f }, 0.0f, Color{ 0.30f, 0.32f, 0.36f, 1.0f });
        AddStaticBox(state, Vec3f{ 0.0f, 8.15f, 0.0f }, Vec3f{ 8.65f, 0.30f, 0.5f }, 0.0f, Color{ 0.30f, 0.32f, 0.36f, 1.0f });
        AddStaticBox(state, Vec3f{ -3.5f, 1.05f, 0.0f }, Vec3f{ 1.25f, 0.12f, 0.5f }, -0.25f, Color{ 0.42f, 0.43f, 0.48f, 1.0f });
        AddStaticBox(state, Vec3f{ 3.35f, 2.1f, 0.0f }, Vec3f{ 1.35f, 0.12f, 0.5f }, 0.28f, Color{ 0.42f, 0.43f, 0.48f, 1.0f });
        AddStaticBox(state, Vec3f{ 0.0f, 4.2f, 0.0f }, Vec3f{ 1.0f, 0.1f, 0.5f }, 0.0f, Color{ 0.42f, 0.43f, 0.48f, 1.0f });
        AddTrigger(state, Vec3f{ 0.0f, 1.75f, 0.0f }, Vec3f{ 0.9f, 0.08f, 0.5f }, Color{ 0.15f, 0.85f, 0.65f, 0.25f });

        const BodyID blue =
            AddSphere(state, Vec3f{ -0.95f, 4.55f, 0.0f }, 0.42f, Color{ 0.31f, 0.68f, 0.96f, 1.0f }).Body;

        const BodyID green =
            AddSphere(state, Vec3f{ -0.10f, 5.10f, 0.0f }, 0.36f, Color{ 0.32f, 0.88f, 0.55f, 1.0f }).Body;

        const BodyID amber =
            AddBox(state, Vec3f{ 0.65f, 4.60f, 0.0f }, Vec3f{ 0.45f, 0.45f, 0.5f }, Color{ 0.98f, 0.72f, 0.25f, 1.0f }).Body;

        const BodyID magenta =
            AddBox(state, Vec3f{ 1.25f, 5.25f, 0.0f }, Vec3f{ 0.55f, 0.32f, 0.5f }, Color{ 0.92f, 0.45f, 0.84f, 1.0f }).Body;

        state.World.Bodies().at(blue).State.LinearVelocity = Vec3f{ 1.0f, 0.0f, 0.0f };
        state.World.Bodies().at(green).State.LinearVelocity = Vec3f{ 0.55f, -0.1f, 0.0f };
        state.World.Bodies().at(amber).State.LinearVelocity = Vec3f{ -0.7f, 0.0f, 0.0f };
        state.World.Bodies().at(magenta).State.LinearVelocity = Vec3f{ -0.45f, -0.05f, 0.0f };

        state.Selected =
            state.Actors.size() - 4u;
    }

    [[nodiscard]]
    Vec3f ActorPosition(
        const SandboxState& state,
        const Actor& actor)
    {
        return state.World.Bodies().at(actor.Body).State.Position;
    }

    [[nodiscard]]
    Quaternionf ActorRotation(
        const SandboxState& state,
        const Actor& actor)
    {
        return (state.World.Bodies().at(actor.Body).State.Rotation * actor.LocalRotation).Normalized();
    }

    void RemoveSelected(
        SandboxState& state)
    {
        Actor* actor =
            SelectedActor(state);

        if (!actor)
        {
            return;
        }

        state.World.DestroyRigidBody(actor->Body);
        SelectNext(state, 1);
    }

    void ClearDynamicActors(
        SandboxState& state)
    {
        for (Actor& actor : state.Actors)
        {
            if (actor.Selectable && ActorAlive(state, actor))
            {
                state.World.DestroyRigidBody(actor.Body);
            }
        }

        state.Selected = 0;
    }

    void SpawnNextSphere(
        SandboxState& state)
    {
        const float x =
            -5.0f + static_cast<float>((state.SpawnIndex * 37) % 100) / 100.0f * 10.0f;

        const float radius =
            state.SpawnIndex % 2 == 0 ? 0.35f : 0.48f;

        AddSphere(
            state,
            Vec3f{ x, 6.55f, 0.0f },
            radius,
            state.SpawnIndex % 2 == 0
                ? Color{ 0.34f, 0.75f, 0.98f, 1.0f }
                : Color{ 0.30f, 0.90f, 0.60f, 1.0f });

        ++state.SpawnIndex;
    }

    void SpawnNextBox(
        SandboxState& state)
    {
        const float x =
            -5.0f + static_cast<float>((state.SpawnIndex * 53) % 100) / 100.0f * 10.0f;

        const Vec3f halfExtents =
            state.SpawnIndex % 2 == 0
                ? Vec3f{ 0.42f, 0.42f, 0.5f }
                : Vec3f{ 0.58f, 0.30f, 0.5f };

        AddBox(
            state,
            Vec3f{ x, 6.55f, 0.0f },
            halfExtents,
            state.SpawnIndex % 2 == 0
                ? Color{ 0.98f, 0.73f, 0.25f, 1.0f }
                : Color{ 0.92f, 0.48f, 0.84f, 1.0f });

        ++state.SpawnIndex;
    }

    void SpawnStack(
        SandboxState& state)
    {
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4 - row; ++col)
            {
                const float x =
                    -0.9f + static_cast<float>(col) * 0.58f + static_cast<float>(row) * 0.29f;

                AddBox(
                    state,
                    Vec3f{ x, 0.45f + static_cast<float>(row) * 0.58f, 0.0f },
                    Vec3f{ 0.26f, 0.26f, 0.5f },
                    Color{ 0.96f, 0.62f, 0.25f, 1.0f });
            }
        }
    }

    void SpawnCollisionDemo(
        SandboxState& state)
    {
        const BodyID leftSphere =
            AddSphere(
                state,
                Vec3f{ -4.4f, 5.15f, 0.0f },
                0.46f,
                Color{ 0.28f, 0.78f, 1.0f, 1.0f }).Body;

        const BodyID rightSphere =
            AddSphere(
                state,
                Vec3f{ 4.4f, 5.15f, 0.0f },
                0.46f,
                Color{ 0.28f, 0.96f, 0.58f, 1.0f }).Body;

        const BodyID leftBox =
            AddBox(
                state,
                Vec3f{ -4.8f, 3.25f, 0.0f },
                Vec3f{ 0.50f, 0.38f, 0.5f },
                Color{ 1.0f, 0.72f, 0.22f, 1.0f }).Body;

        const BodyID rightBox =
            AddBox(
                state,
                Vec3f{ 4.8f, 3.25f, 0.0f },
                Vec3f{ 0.50f, 0.38f, 0.5f },
                Color{ 0.94f, 0.42f, 0.92f, 1.0f }).Body;

        RigidBody& leftSphereBody =
            state.World.Bodies().at(leftSphere);

        RigidBody& rightSphereBody =
            state.World.Bodies().at(rightSphere);

        RigidBody& leftBoxBody =
            state.World.Bodies().at(leftBox);

        RigidBody& rightBoxBody =
            state.World.Bodies().at(rightBox);

        leftSphereBody.EnableGravity = false;
        rightSphereBody.EnableGravity = false;
        leftBoxBody.EnableGravity = false;
        rightBoxBody.EnableGravity = false;

        leftSphereBody.LinearDamping = 0.03f;
        rightSphereBody.LinearDamping = 0.03f;
        leftBoxBody.LinearDamping = 0.03f;
        rightBoxBody.LinearDamping = 0.03f;

        leftSphereBody.State.LinearVelocity = Vec3f{ CollisionDemoSpeed, 0.0f, 0.0f };
        rightSphereBody.State.LinearVelocity = Vec3f{ -CollisionDemoSpeed, 0.0f, 0.0f };
        leftBoxBody.State.LinearVelocity = Vec3f{ CollisionDemoSpeed * 0.95f, 0.0f, 0.0f };
        rightBoxBody.State.LinearVelocity = Vec3f{ -CollisionDemoSpeed * 0.95f, 0.0f, 0.0f };
    }

    [[nodiscard]]
    float ActorContainmentRadius(
        const Actor& actor)
    {
        if (actor.Kind == ActorKind::Sphere)
        {
            return actor.Radius;
        }

        return std::sqrt(
            actor.HalfExtents.x * actor.HalfExtents.x +
            actor.HalfExtents.y * actor.HalfExtents.y) + 0.05f;
    }

    void EnforcePlayBounds(
        SandboxState& state)
    {
        for (Actor& actor : state.Actors)
        {
            if (!actor.Selectable || !ActorAlive(state, actor))
            {
                continue;
            }

            RigidBody& body =
                state.World.Bodies().at(actor.Body);

            const float radius =
                ActorContainmentRadius(actor);

            Vec3f position =
                body.State.Position;

            bool correctedX =
                false;

            bool correctedY =
                false;

            const float minX =
                PlayLeft + radius;

            const float maxX =
                PlayRight - radius;

            const float minY =
                PlayBottom + radius;

            const float maxY =
                PlayTop - radius;

            if (position.x < minX)
            {
                position.x = minX;
                correctedX = true;
            }
            else if (position.x > maxX)
            {
                position.x = maxX;
                correctedX = true;
            }

            if (position.y < minY)
            {
                position.y = minY;
                correctedY = true;
            }
            else if (position.y > maxY)
            {
                position.y = maxY;
                correctedY = true;
            }

            if (correctedX || correctedY)
            {
                body.State.Position = position;
                if (correctedX)
                {
                    body.State.LinearVelocity.x = 0.0f;
                }
                if (correctedY)
                {
                    body.State.LinearVelocity.y = 0.0f;
                }
                state.World.WakeBody(actor.Body);
            }
        }
    }

    void SetColor(
        const Color& color)
    {
        glColor4f(color.R, color.G, color.B, color.A);
    }

    void DrawLine(
        const Vec3f& a,
        const Vec3f& b,
        const Color& color,
        float width = 1.0f)
    {
        glLineWidth(width);
        SetColor(color);
        glBegin(GL_LINES);
        glVertex2f(a.x, a.y);
        glVertex2f(b.x, b.y);
        glEnd();
        glLineWidth(1.0f);
    }

    void DrawFilledCircle(
        const Vec3f& center,
        float radius,
        const Color& fill,
        const Color& outline,
        bool selected)
    {
        SetColor(fill);
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(center.x, center.y);
        for (int i = 0; i <= 72; ++i)
        {
            const float angle =
                (static_cast<float>(i) / 72.0f) * Pi * 2.0f;

            glVertex2f(
                center.x + std::cos(angle) * radius,
                center.y + std::sin(angle) * radius);
        }
        glEnd();

        glLineWidth(selected ? 4.0f : 2.0f);
        SetColor(selected ? Color{ 1.0f, 1.0f, 1.0f, 1.0f } : outline);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 72; ++i)
        {
            const float angle =
                (static_cast<float>(i) / 72.0f) * Pi * 2.0f;

            glVertex2f(
                center.x + std::cos(angle) * radius,
                center.y + std::sin(angle) * radius);
        }
        glEnd();
        glLineWidth(1.0f);
    }

    void DrawBox(
        const Vec3f& center,
        const Quaternionf& rotation,
        const Vec3f& halfExtents,
        const Color& fill,
        const Color& outline,
        bool selected)
    {
        const Vec3f local[4]
        {
            Vec3f{ -halfExtents.x, -halfExtents.y, 0.0f },
            Vec3f{ halfExtents.x, -halfExtents.y, 0.0f },
            Vec3f{ halfExtents.x, halfExtents.y, 0.0f },
            Vec3f{ -halfExtents.x, halfExtents.y, 0.0f }
        };

        Vec3f world[4];
        for (int i = 0; i < 4; ++i)
        {
            world[i] =
                center + Rotate(rotation, local[i]);
        }

        SetColor(fill);
        glBegin(GL_QUADS);
        for (const Vec3f& point : world)
        {
            glVertex2f(point.x, point.y);
        }
        glEnd();

        glLineWidth(selected ? 4.0f : 2.0f);
        SetColor(selected ? Color{ 1.0f, 1.0f, 1.0f, 1.0f } : outline);
        glBegin(GL_LINE_LOOP);
        for (const Vec3f& point : world)
        {
            glVertex2f(point.x, point.y);
        }
        glEnd();
        glLineWidth(1.0f);

        DrawLine(
            center,
            center + Rotate(rotation, Vec3f::UnitX()) * halfExtents.x,
            Color{ 0.98f, 0.24f, 0.24f, 1.0f },
            1.5f);
    }

    void DrawGrid()
    {
        for (int x = static_cast<int>(ViewLeft); x <= static_cast<int>(ViewRight); ++x)
        {
            DrawLine(
                Vec3f{ static_cast<float>(x), ViewBottom, 0.0f },
                Vec3f{ static_cast<float>(x), ViewTop, 0.0f },
                Color{ 0.16f, 0.17f, 0.19f, 1.0f });
        }

        for (int y = static_cast<int>(ViewBottom); y <= static_cast<int>(ViewTop); ++y)
        {
            DrawLine(
                Vec3f{ ViewLeft, static_cast<float>(y), 0.0f },
                Vec3f{ ViewRight, static_cast<float>(y), 0.0f },
                Color{ 0.16f, 0.17f, 0.19f, 1.0f });
        }
    }

    void DrawVelocity(
        const SandboxState& state,
        const Actor& actor)
    {
        const MotionState& motion =
            state.World.Bodies().at(actor.Body).State;

        DrawLine(
            motion.Position,
            motion.Position + motion.LinearVelocity * 0.12f,
            Color{ 0.40f, 1.0f, 0.58f, 1.0f },
            2.0f);
    }

    void DrawDebugAABBs(
        const SandboxState& state)
    {
        for (const DebugAABB& bounds : state.World.DebugAABBs())
        {
            const Vec3f min =
                bounds.Bounds.Min;

            const Vec3f max =
                bounds.Bounds.Max;

            glLineWidth(1.0f);
            SetColor(Color{ 0.30f, 0.50f, 1.0f, 0.35f });
            glBegin(GL_LINE_LOOP);
            glVertex2f(min.x, min.y);
            glVertex2f(max.x, min.y);
            glVertex2f(max.x, max.y);
            glVertex2f(min.x, max.y);
            glEnd();
        }
    }

    void DrawRayFan(
        SandboxState& state,
        const Actor* selected)
    {
        if (!state.ShowRays)
        {
            return;
        }

        const Vec3f origin =
            selected && ActorAlive(state, *selected)
                ? ActorPosition(state, *selected)
                : Vec3f{ 0.0f, 4.0f, 0.0f };

        const ColliderID ignoredCollider =
            selected && ActorAlive(state, *selected)
                ? selected->Collider
                : InvalidColliderID;

        DrawFilledCircle(
            origin,
            0.075f,
            Color{ 1.0f, 0.86f, 0.20f, 1.0f },
            Color{ 1.0f, 0.86f, 0.20f, 1.0f },
            false);

        for (int i = 0; i < RayFanCount; ++i)
        {
            const float fraction =
                RayFanCount == 1 ? 0.5f : static_cast<float>(i) / static_cast<float>(RayFanCount - 1);

            const float angle =
                state.RayAngle + (fraction - 0.5f) * RayFanSpread;

            const Vec3f direction =
                SafeNormalize(Vec3f{ std::cos(angle), std::sin(angle), 0.0f }, Vec3f::UnitX());

            const std::vector<PhysicsRayHit> hits =
                state.World.RaycastAll(
                    origin,
                    direction,
                    RayMaxDistance,
                    0xFFFF'FFFFu,
                    ignoredCollider);

            const Vec3f end =
                hits.empty()
                    ? origin + direction * RayMaxDistance
                    : hits.front().Point;

            DrawLine(
                origin,
                end,
                hits.empty()
                    ? Color{ 1.0f, 0.78f, 0.16f, 0.30f }
                    : Color{ 1.0f, 0.85f, 0.20f, 0.70f },
                hits.empty() ? 1.0f : 2.0f);

            const std::size_t markerCount =
                std::min<std::size_t>(hits.size(), 3u);

            for (std::size_t hitIndex = 0; hitIndex < markerCount; ++hitIndex)
            {
                const PhysicsRayHit& hit =
                    hits.at(hitIndex);

                DrawFilledCircle(
                    hit.Point,
                    hitIndex == 0 ? 0.060f : 0.040f,
                    hit.IsTrigger
                        ? Color{ 0.24f, 1.0f, 0.70f, 0.90f }
                        : Color{ 1.0f, 0.38f, 0.18f, 0.95f },
                    Color{ 0.08f, 0.06f, 0.03f, 1.0f },
                    false);
            }

            if (!hits.empty())
            {
                const PhysicsRayHit& first =
                    hits.front();

                DrawLine(
                    first.Point,
                    first.Point + first.Normal * 0.38f,
                    Color{ 1.0f, 0.16f, 0.12f, 0.95f },
                    2.0f);

                const Vec3f reflection =
                    SafeNormalize(
                        direction - first.Normal * (2.0f * Dot(direction, first.Normal)),
                        direction);

                DrawLine(
                    first.Point,
                    first.Point + reflection * 0.75f,
                    Color{ 0.20f, 0.95f, 1.0f, 0.65f },
                    1.5f);
            }
        }
    }

    void Render(
        SandboxState& state,
        int width,
        int height)
    {
        glViewport(0, 0, width, height);
        glClearColor(0.055f, 0.060f, 0.070f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(ViewLeft, ViewRight, ViewBottom, ViewTop, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        DrawGrid();

        Actor* selected =
            SelectedActor(state);

        for (const Actor& actor : state.Actors)
        {
            if (!ActorAlive(state, actor))
            {
                continue;
            }

            const bool isSelected =
                selected && selected->Body == actor.Body;

            const Vec3f position =
                ActorPosition(state, actor);

            const Quaternionf rotation =
                ActorRotation(state, actor);

            if (actor.Kind == ActorKind::Sphere)
            {
                DrawFilledCircle(
                    position,
                    actor.Radius,
                    actor.Fill,
                    Color{ 0.05f, 0.08f, 0.10f, 1.0f },
                    isSelected);
            }
            else
            {
                DrawBox(
                    position,
                    rotation,
                    actor.HalfExtents,
                    actor.Fill,
                    actor.Kind == ActorKind::Trigger
                        ? Color{ 0.30f, 1.0f, 0.75f, 0.9f }
                        : Color{ 0.05f, 0.06f, 0.08f, 1.0f },
                    isSelected);
            }

            if (actor.Selectable)
            {
                DrawVelocity(state, actor);
            }
        }

        DrawRayFan(state, selected);

        for (const DebugContact& contact : state.World.DebugContacts())
        {
            DrawFilledCircle(contact.Position, 0.045f, Color{ 1.0f, 0.18f, 0.18f, 1.0f }, Color{ 1.0f, 0.18f, 0.18f, 1.0f }, false);
            DrawLine(
                contact.Position,
                contact.Position + contact.Normal * 0.35f,
                Color{ 1.0f, 0.20f, 0.20f, 1.0f },
                2.0f);
        }

        if (state.ShowDebug)
        {
            DrawDebugAABBs(state);
        }
    }

    [[nodiscard]]
    bool KeyDown(
        GLFWwindow* window,
        int key)
    {
        return glfwGetKey(window, key) == GLFW_PRESS;
    }

    void MoveSelected(
        SandboxState& state,
        GLFWwindow* window,
        float dt)
    {
        Actor* actor =
            SelectedActor(state);

        if (!actor || !actor->Selectable || !ActorAlive(state, *actor))
        {
            return;
        }

        RigidBody& body =
            state.World.Bodies().at(actor->Body);

        Vec3f direction =
            Vec3f::Zero();

        if (KeyDown(window, GLFW_KEY_LEFT) || KeyDown(window, GLFW_KEY_A))
        {
            direction.x -= 1.0f;
        }

        if (KeyDown(window, GLFW_KEY_RIGHT) || KeyDown(window, GLFW_KEY_D))
        {
            direction.x += 1.0f;
        }

        if (KeyDown(window, GLFW_KEY_UP) || KeyDown(window, GLFW_KEY_W))
        {
            direction.y += 1.0f;
        }

        if (KeyDown(window, GLFW_KEY_DOWN) || KeyDown(window, GLFW_KEY_S))
        {
            direction.y -= 1.0f;
        }

        const bool directMove =
            KeyDown(window, GLFW_KEY_LEFT_SHIFT) ||
            KeyDown(window, GLFW_KEY_RIGHT_SHIFT);

        if (direction.LengthSquared() > 1.0e-6f)
        {
            direction =
                SafeNormalize(direction, Vec3f::UnitX());

            if (directMove)
            {
                body.State.Position += direction * (DirectMoveSpeed * dt);
                body.State.LinearVelocity = Vec3f::Zero();
                state.World.WakeBody(actor->Body);
            }
            else
            {
                const float mass =
                    body.Mass.InverseMass > 0.0f
                        ? 1.0f / body.Mass.InverseMass
                        : 1.0f;

                state.World.AddBodyForce(actor->Body, direction * (PushAcceleration * mass));
            }
        }

        if (KeyDown(window, GLFW_KEY_Q))
        {
            state.World.AddBodyTorque(actor->Body, Vec3f{ 0.0f, 0.0f, TorqueAcceleration });
        }

        if (KeyDown(window, GLFW_KEY_E))
        {
            state.World.AddBodyTorque(actor->Body, Vec3f{ 0.0f, 0.0f, -TorqueAcceleration });
        }

        EnforcePlayBounds(state);
    }

    [[nodiscard]]
    Vec3f MouseToWorld(
        GLFWwindow* window,
        double mouseX,
        double mouseY)
    {
        int width = 1;
        int height = 1;
        glfwGetFramebufferSize(window, &width, &height);

        const float x =
            ViewLeft + static_cast<float>(mouseX) / static_cast<float>(width) * (ViewRight - ViewLeft);

        const float y =
            ViewTop - static_cast<float>(mouseY) / static_cast<float>(height) * (ViewTop - ViewBottom);

        return Vec3f{ x, y, 0.0f };
    }

    void SelectAtMouse(
        SandboxState& state,
        GLFWwindow* window)
    {
        double mouseX = 0.0;
        double mouseY = 0.0;
        glfwGetCursorPos(window, &mouseX, &mouseY);

        const Vec3f point =
            MouseToWorld(window, mouseX, mouseY);

        float bestDistanceSq =
            0.45f;

        std::size_t best =
            state.Selected;

        bool found =
            false;

        for (std::size_t i = 0; i < state.Actors.size(); ++i)
        {
            const Actor& actor =
                state.Actors.at(i);

            if (!actor.Selectable || !ActorAlive(state, actor))
            {
                continue;
            }

            const float distanceSq =
                (ActorPosition(state, actor) - point).LengthSquared();

            if (distanceSq < bestDistanceSq)
            {
                bestDistanceSq = distanceSq;
                best = i;
                found = true;
            }
        }

        if (found)
        {
            state.Selected = best;
        }
    }

    void UpdateTitle(
        GLFWwindow* window,
        SandboxState& state)
    {
        const Actor* selected =
            SelectedActor(state);

        const char* kind =
            "none";

        BodyID body =
            InvalidBodyID;

        if (selected)
        {
            body = selected->Body;
            switch (selected->Kind)
            {
            case ActorKind::Sphere:
                kind = "sphere";
                break;
            case ActorKind::Box:
                kind = "box";
                break;
            case ActorKind::StaticBox:
                kind = "static";
                break;
            case ActorKind::Trigger:
                kind = "trigger";
                break;
            }
        }

        const std::size_t activeCount =
            static_cast<std::size_t>(
                std::count_if(
                    state.Actors.begin(),
                    state.Actors.end(),
                    [&state](const Actor& actor)
                    {
                        return actor.Selectable && ActorAlive(state, actor);
                    }));

        char title[512];
        std::snprintf(
            title,
            sizeof(title),
            "KairoPhysics Playground | %s | gravity %s | rays %s | selected %u %s | dynamic %zu | contacts %zu events %zu callbacks %u trigger-cb %u | 1 sphere 2 box 3 stack 4 collision L rays J/K rotate rays Delete remove C clear Tab next [ previous WASD/Arrows gentle push Shift+move Q/E torque Space pause N step R reset G gravity T debug",
            state.Paused ? "paused" : "running",
            state.GravityEnabled ? "on" : "off",
            state.ShowRays ? "on" : "off",
            body,
            kind,
            activeCount,
            state.World.Contacts().size(),
            state.World.ContactEvents().size(),
            state.CallbackEvents,
            state.TriggerCallbackEvents);

        glfwSetWindowTitle(window, title);
    }
}

int main()
{
    if (glfwInit() != GLFW_TRUE)
    {
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> window(
        glfwCreateWindow(1280, 760, "KairoPhysics Playground", nullptr, nullptr),
        glfwDestroyWindow);

    if (!window)
    {
        glfwTerminate();
        return 2;
    }

    glfwMakeContextCurrent(window.get());
    glfwSwapInterval(1);

    SandboxState state;
    ResetWorld(state);

    auto previous =
        std::chrono::steady_clock::now();

    KeyLatch space;
    KeyLatch reset;
    KeyLatch debug;
    KeyLatch gravity;
    KeyLatch selectNext;
    KeyLatch selectPrevious;
    KeyLatch spawnSphere;
    KeyLatch spawnBox;
    KeyLatch spawnStack;
    KeyLatch spawnCollision;
    KeyLatch remove;
    KeyLatch clear;
    KeyLatch toggleRays;
    KeyLatch singleStep;
    KeyLatch mouseSelect;

    while (!glfwWindowShouldClose(window.get()))
    {
        glfwPollEvents();

        if (KeyDown(window.get(), GLFW_KEY_ESCAPE))
        {
            glfwSetWindowShouldClose(window.get(), GLFW_TRUE);
        }

        const auto now =
            std::chrono::steady_clock::now();

        const float elapsed =
            std::chrono::duration<float>(now - previous).count();

        previous = now;

        if (space.Pressed(KeyDown(window.get(), GLFW_KEY_SPACE)))
        {
            state.Paused = !state.Paused;
        }

        if (reset.Pressed(KeyDown(window.get(), GLFW_KEY_R)))
        {
            ResetWorld(state);
        }

        if (debug.Pressed(KeyDown(window.get(), GLFW_KEY_T)))
        {
            state.ShowDebug = !state.ShowDebug;
        }

        if (gravity.Pressed(KeyDown(window.get(), GLFW_KEY_G)))
        {
            state.GravityEnabled = !state.GravityEnabled;
            state.World.Gravity = state.GravityEnabled ? DefaultGravity : Vec3f::Zero();
        }

        if (selectNext.Pressed(KeyDown(window.get(), GLFW_KEY_TAB)))
        {
            SelectNext(state, 1);
        }

        if (selectPrevious.Pressed(KeyDown(window.get(), GLFW_KEY_LEFT_BRACKET)))
        {
            SelectNext(state, -1);
        }

        if (spawnSphere.Pressed(KeyDown(window.get(), GLFW_KEY_1)))
        {
            SpawnNextSphere(state);
        }

        if (spawnBox.Pressed(KeyDown(window.get(), GLFW_KEY_2)))
        {
            SpawnNextBox(state);
        }

        if (spawnStack.Pressed(KeyDown(window.get(), GLFW_KEY_3)))
        {
            SpawnStack(state);
        }

        if (spawnCollision.Pressed(KeyDown(window.get(), GLFW_KEY_4)))
        {
            SpawnCollisionDemo(state);
        }

        if (toggleRays.Pressed(KeyDown(window.get(), GLFW_KEY_L)))
        {
            state.ShowRays = !state.ShowRays;
        }

        if (remove.Pressed(KeyDown(window.get(), GLFW_KEY_DELETE) || KeyDown(window.get(), GLFW_KEY_BACKSPACE)))
        {
            RemoveSelected(state);
        }

        if (clear.Pressed(KeyDown(window.get(), GLFW_KEY_C)))
        {
            ClearDynamicActors(state);
        }

        if (mouseSelect.Pressed(glfwGetMouseButton(window.get(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS))
        {
            SelectAtMouse(state, window.get());
        }

        MoveSelected(state, window.get(), std::min(elapsed, 1.0f / 30.0f));

        if (KeyDown(window.get(), GLFW_KEY_J))
        {
            state.RayAngle += RayRotationSpeed * std::min(elapsed, 1.0f / 30.0f);
        }

        if (KeyDown(window.get(), GLFW_KEY_K))
        {
            state.RayAngle -= RayRotationSpeed * std::min(elapsed, 1.0f / 30.0f);
        }

        const bool stepOnce =
            singleStep.Pressed(KeyDown(window.get(), GLFW_KEY_N));

        if (!state.Paused || stepOnce)
        {
            state.World.StepFixed(
                stepOnce ? (1.0f / 60.0f) : std::min(elapsed, 1.0f / 20.0f),
                1.0f / 60.0f,
                5);
        }

        EnforcePlayBounds(state);

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window.get(), &width, &height);

        Render(state, width, height);
        UpdateTitle(window.get(), state);
        glfwSwapBuffers(window.get());
    }

    glfwTerminate();
    return 0;
}
