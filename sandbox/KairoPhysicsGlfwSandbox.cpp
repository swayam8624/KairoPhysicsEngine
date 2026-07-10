#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>

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
    struct SandboxState final
    {
        PhysicsWorld World;
        BodyID DynamicBody = InvalidBodyID;
        bool Paused = false;
        bool UseBox = false;
    };

    RigidBodyDesc DynamicSphereBody(
        const Vec3f& position)
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Dynamic;
        desc.State.Position = position;
        desc.Mass = SphereMassProperties(0.5f, 1.0f);
        desc.LinearDamping = 0.02f;
        desc.AngularDamping = 0.02f;
        desc.AllowSleeping = false;
        return desc;
    }

    RigidBodyDesc DynamicBoxBody(
        const Vec3f& position)
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Dynamic;
        desc.State.Position = position;
        desc.State.Rotation = RotationAroundZ(0.35f);
        desc.Mass = BoxMassProperties(Vec3f{ 0.5f, 0.5f, 0.5f }, 1.0f);
        desc.LinearDamping = 0.02f;
        desc.AngularDamping = 0.02f;
        desc.AllowSleeping = false;
        return desc;
    }

    RigidBodyDesc StaticBody()
    {
        RigidBodyDesc desc;
        desc.Type = BodyType::Static;
        return desc;
    }

    void ResetWorld(
        SandboxState& state)
    {
        state.World = PhysicsWorld{};
        state.World.Settings.VelocityIterations = 16;
        state.World.Settings.PositionIterations = 6;
        state.World.Settings.EnableSleeping = false;

        if (state.UseBox)
        {
            state.DynamicBody =
                state.World.CreateRigidBody(DynamicBoxBody(Vec3f{ 0.0f, 3.0f, 0.0f }));

            [[maybe_unused]] const ColliderID boxCollider =
                state.World.AddCollider(
                state.DynamicBody,
                BoxCollider{ Vec3f{ 0.5f, 0.5f, 0.5f } },
                {},
                Vec3f::Zero(),
                RotationAroundZ(0.2f));
        }
        else
        {
            state.DynamicBody =
                state.World.CreateRigidBody(DynamicSphereBody(Vec3f{ 0.0f, 3.0f, 0.0f }));

            [[maybe_unused]] const ColliderID sphereCollider =
                state.World.AddCollider(state.DynamicBody, SphereCollider{ 0.5f });
        }

        const BodyID floor =
            state.World.CreateRigidBody(StaticBody());

        [[maybe_unused]] const ColliderID floorCollider =
            state.World.AddCollider(floor, PlaneCollider{ Vec3f::Up(), 0.0f });
    }

    void DrawLine(
        const Vec3f& a,
        const Vec3f& b,
        float r,
        float g,
        float bl)
    {
        glColor3f(r, g, bl);
        glBegin(GL_LINES);
        glVertex2f(a.x, a.y);
        glVertex2f(b.x, b.y);
        glEnd();
    }

    void DrawCircle(
        const Vec3f& center,
        float radius,
        float r,
        float g,
        float bl)
    {
        glColor3f(r, g, bl);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 64; ++i)
        {
            const float angle =
                (static_cast<float>(i) / 64.0f) * 6.28318530718f;

            glVertex2f(
                center.x + std::cos(angle) * radius,
                center.y + std::sin(angle) * radius);
        }
        glEnd();
    }

    void DrawBox(
        const RigidBody& body,
        float halfExtent,
        float r,
        float g,
        float bl)
    {
        const Vec3f local[4]
        {
            Vec3f{ -halfExtent, -halfExtent, 0.0f },
            Vec3f{ halfExtent, -halfExtent, 0.0f },
            Vec3f{ halfExtent, halfExtent, 0.0f },
            Vec3f{ -halfExtent, halfExtent, 0.0f }
        };

        glColor3f(r, g, bl);
        glBegin(GL_LINE_LOOP);
        for (const Vec3f& corner : local)
        {
            const Vec3f world =
                body.State.Position + Rotate(body.State.Rotation, corner);
            glVertex2f(world.x, world.y);
        }
        glEnd();
    }

    void Render(
        const SandboxState& state,
        int width,
        int height)
    {
        glViewport(0, 0, width, height);
        glClearColor(0.07f, 0.08f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-4.0, 4.0, -0.5, 5.0, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        DrawLine(Vec3f{ -4.0f, 0.0f, 0.0f }, Vec3f{ 4.0f, 0.0f, 0.0f }, 0.45f, 0.48f, 0.52f);

        const RigidBody& body =
            state.World.Bodies().at(state.DynamicBody);

        if (state.UseBox)
        {
            DrawBox(body, 0.5f, 0.95f, 0.76f, 0.35f);
        }
        else
        {
            DrawCircle(body.State.Position, 0.5f, 0.35f, 0.75f, 0.95f);
        }

        for (const DebugContact& contact : state.World.DebugContacts())
        {
            DrawLine(
                contact.Position,
                contact.Position + contact.Normal * 0.35f,
                1.0f,
                0.25f,
                0.25f);
        }
    }

    void UpdateTitle(
        GLFWwindow* window,
        const SandboxState& state)
    {
        const MotionState& motion =
            state.World.Bodies().at(state.DynamicBody).State;

        char title[256];
        std::snprintf(
            title,
            sizeof(title),
            "KairoPhysicsGlfwSandbox | %s | %s | y=%.3f vy=%.3f contacts=%zu | Space pause, R reset, B toggle",
            state.UseBox ? "box" : "sphere",
            state.Paused ? "paused" : "running",
            motion.Position.y,
            motion.LinearVelocity.y,
            state.World.Contacts().size());

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
        glfwCreateWindow(960, 640, "KairoPhysicsGlfwSandbox", nullptr, nullptr),
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

    bool spaceWasDown = false;
    bool resetWasDown = false;
    bool boxWasDown = false;

    while (!glfwWindowShouldClose(window.get()))
    {
        glfwPollEvents();

        if (glfwGetKey(window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS)
        {
            glfwSetWindowShouldClose(window.get(), GLFW_TRUE);
        }

        const bool spaceDown =
            glfwGetKey(window.get(), GLFW_KEY_SPACE) == GLFW_PRESS;
        const bool resetDown =
            glfwGetKey(window.get(), GLFW_KEY_R) == GLFW_PRESS;
        const bool boxDown =
            glfwGetKey(window.get(), GLFW_KEY_B) == GLFW_PRESS;

        if (spaceDown && !spaceWasDown)
        {
            state.Paused = !state.Paused;
        }

        if (resetDown && !resetWasDown)
        {
            ResetWorld(state);
        }

        if (boxDown && !boxWasDown)
        {
            state.UseBox = !state.UseBox;
            ResetWorld(state);
        }

        spaceWasDown = spaceDown;
        resetWasDown = resetDown;
        boxWasDown = boxDown;

        const auto now =
            std::chrono::steady_clock::now();

        const float elapsed =
            std::chrono::duration<float>(now - previous).count();

        previous = now;

        if (!state.Paused)
        {
            state.World.StepFixed(elapsed, 1.0f / 60.0f, 4);
        }

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
