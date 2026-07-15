cmake_minimum_required(VERSION 3.28)
project(KairoPhysicsEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(KAIRO_PHYSICS_ENGINE_BUILD_TESTS "Build KairoPhysicsEngine tests" ${PROJECT_IS_TOP_LEVEL})
option(KAIRO_PHYSICS_ENGINE_BUILD_SANDBOX "Build terminal KairoPhysicsEngine sandbox" ${PROJECT_IS_TOP_LEVEL})
option(KAIRO_PHYSICS_ENGINE_BUILD_GLFW_SANDBOX "Build GLFW KairoPhysicsEngine debug sandbox when GLFW is available" ${PROJECT_IS_TOP_LEVEL})

# Immutable standalone fallback revisions. The superproject supplies local
# targets; these pins make a direct clone deterministic when it does not.
set(KAIRO_PHYSICS_ENGINE_PHYSICS_MATH_REVISION 88df8508a4c311a731226056d79054a859466e8c)
set(KAIRO_PHYSICS_ENGINE_SPATIAL_REVISION a5ef985597ce554fd3328b11e58c7c7be0edcace)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
elseif(MSVC)
    add_compile_options(/W4)
endif()

if(NOT TARGET KairoPhysicsMath)
    set(KAIRO_PHYSICS_MATH_SOURCE_DIR "" CACHE PATH "Path to a local KairoPhysicsMath checkout")

    if(KAIRO_PHYSICS_MATH_SOURCE_DIR AND EXISTS "${KAIRO_PHYSICS_MATH_SOURCE_DIR}/CMakeLists.txt")
        add_subdirectory("${KAIRO_PHYSICS_MATH_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoPhysicsMath")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../KairoPhysicsMath/CMakeLists.txt")
        add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../KairoPhysicsMath" "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoPhysicsMath")
    else()
        include(FetchContent)
        FetchContent_Declare(
            KairoPhysicsMath
            GIT_REPOSITORY https://github.com/swayam8624/KairoPhysicsMath.git
            GIT_TAG ${KAIRO_PHYSICS_ENGINE_PHYSICS_MATH_REVISION}
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(KairoPhysicsMath)
    endif()
endif()

if(NOT TARGET KairoSpatial)
    set(KAIRO_SPATIAL_SOURCE_DIR "" CACHE PATH "Path to a local KairoSpatial checkout")

    if(KAIRO_SPATIAL_SOURCE_DIR AND EXISTS "${KAIRO_SPATIAL_SOURCE_DIR}/CMakeLists.txt")
        add_subdirectory("${KAIRO_SPATIAL_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoSpatial")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../Spatial/CMakeLists.txt")
        add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../Spatial" "${CMAKE_CURRENT_BINARY_DIR}/_deps/KairoSpatial")
    else()
        include(FetchContent)
        FetchContent_Declare(
            KairoSpatial
            GIT_REPOSITORY https://github.com/swayam8624/KairoSpatial.git
            GIT_TAG ${KAIRO_PHYSICS_ENGINE_SPATIAL_REVISION}
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(KairoSpatial)
    endif()
endif()

add_library(KairoPhysicsEngine)

target_sources(KairoPhysicsEngine
    PUBLIC
        FILE_SET CXX_MODULES
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            PhysicsEngineTypes.cppm
            PhysicsMaterial.cppm
            RigidBody.cppm
            Collider.cppm
            Broadphase.cppm
            Narrowphase.cppm
            ContactSolver.cppm
            PhysicsDebug.cppm
            PhysicsWorld.cppm
            ProjectileSystem.cppm
            Buoyancy.cppm
            KairoPhysicsEngine.cppm
)

target_link_libraries(KairoPhysicsEngine PUBLIC KairoPhysicsMath KairoSpatial)

# The debug window consumes the engine target, so it must be declared only
# after the module library and its dependency graph are complete.
if(KAIRO_PHYSICS_ENGINE_BUILD_GLFW_SANDBOX)
    find_package(glfw3 QUIET)
    find_package(OpenGL QUIET)

    if(glfw3_FOUND AND OpenGL_FOUND)
        add_executable(KairoPhysicsGlfwSandbox sandbox/KairoPhysicsGlfwSandbox.cpp)
        target_link_libraries(KairoPhysicsGlfwSandbox PRIVATE KairoPhysicsEngine glfw OpenGL::GL)
    else()
        message(STATUS "KairoPhysicsGlfwSandbox skipped: GLFW or OpenGL not found.")
    endif()
endif()

if(KAIRO_PHYSICS_ENGINE_BUILD_SANDBOX)
    add_library(KairoPhysicsSandboxCore)
    target_sources(KairoPhysicsSandboxCore
        PUBLIC
            FILE_SET CXX_MODULES
            BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/sandbox
            FILES
                sandbox/SandboxTypes.cppm
                sandbox/DemoScene.cppm
                sandbox/TerminalRenderer.cppm
                sandbox/PhysicsDebugRenderer.cppm
                sandbox/SandboxApp.cppm
                sandbox/KairoPhysicsSandbox.cppm
    )
    target_link_libraries(KairoPhysicsSandboxCore PUBLIC KairoPhysicsEngine)

    add_executable(KairoPhysicsSandbox sandbox/examples/main.cpp)
    target_link_libraries(KairoPhysicsSandbox PRIVATE KairoPhysicsSandboxCore)
endif()

if(KAIRO_PHYSICS_ENGINE_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.4
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(KairoPhysicsEngineTests tests/PhysicsEngineTests.cpp)
    target_link_libraries(KairoPhysicsEngineTests PRIVATE KairoPhysicsEngine Catch2::Catch2WithMain)

    enable_testing()
    add_test(NAME KairoPhysicsEngineTests COMMAND KairoPhysicsEngineTests)
endif()
