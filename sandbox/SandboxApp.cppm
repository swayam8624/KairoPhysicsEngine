module;

#include <charconv>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

export module Kairo.Foundation.PhysicsSandbox.App;

import Kairo.Foundation.PhysicsSandbox.Types;
import Kairo.Foundation.PhysicsSandbox.DemoScene;
import Kairo.Foundation.PhysicsSandbox.TerminalRenderer;

export namespace kairo::foundation::physics::sandbox
{
    namespace detail
    {
        template<typename T>
        [[nodiscard]]
        T ParseNumber(
            std::string_view text,
            const char* optionName)
        {
            T value{};
            const char* begin =
                text.data();

            const char* end =
                text.data() + text.size();

            const std::from_chars_result result =
                std::from_chars(begin, end, value);

            if (result.ec != std::errc{} || result.ptr != end)
            {
                throw std::invalid_argument(
                    std::string(optionName) +
                    " expects a valid numeric value.");
            }

            return value;
        }

        [[nodiscard]]
        std::string_view RequireValue(
            int argc,
            char** argv,
            int& index,
            const char* optionName)
        {
            if (index + 1 >= argc)
            {
                throw std::invalid_argument(
                    std::string(optionName) +
                    " requires a value.");
            }

            ++index;
            return argv[index];
        }

        void PrintUsage(
            std::ostream& out,
            const char* executable)
        {
            out
                << "Usage: " << executable << " [scenario] [options]\n\n"
                << "Scenarios:\n  " << SupportedScenarioList() << "\n\n"
                << "Options:\n"
                << "  --steps N       fixed steps to simulate\n"
                << "  --every N       print every N steps\n"
                << "  --dt SECONDS    fixed timestep, default 1/60\n"
                << "  --csv PATH      write benchmark CSV\n"
                << "  --no-ascii      disable ASCII side-view\n"
                << "  --no-bodies     disable per-body rows\n"
                << "  --help          show this message\n";
        }

        [[nodiscard]]
        SandboxRunSettings ParseArguments(
            int argc,
            char** argv)
        {
            SandboxRunSettings settings;

            int index =
                1;

            if (index < argc && std::string_view(argv[index]).starts_with("--") == false)
            {
                settings.Scenario = ParseScenarioName(argv[index]);
                ++index;
            }

            for (; index < argc; ++index)
            {
                const std::string_view option =
                    argv[index];

                if (option == "--help")
                {
                    PrintUsage(std::cout, argv[0]);
                    throw std::runtime_error("help requested");
                }

                if (option == "--steps")
                {
                    settings.Steps =
                        ParseNumber<std::uint32_t>(
                            RequireValue(argc, argv, index, "--steps"),
                            "--steps");
                }
                else if (option == "--every")
                {
                    settings.PrintEvery =
                        ParseNumber<std::uint32_t>(
                            RequireValue(argc, argv, index, "--every"),
                            "--every");
                }
                else if (option == "--dt")
                {
                    settings.FixedDt =
                        ParseNumber<float>(
                            RequireValue(argc, argv, index, "--dt"),
                            "--dt");
                }
                else if (option == "--csv")
                {
                    settings.CsvPath =
                        std::string(RequireValue(argc, argv, index, "--csv"));
                }
                else if (option == "--no-ascii")
                {
                    settings.ShowAscii = false;
                }
                else if (option == "--no-bodies")
                {
                    settings.PrintBodies = false;
                }
                else
                {
                    throw std::invalid_argument(
                        "Unknown sandbox option '" + std::string(option) + "'.");
                }
            }

            if (settings.Steps == 0)
            {
                throw std::invalid_argument("--steps must be greater than zero.");
            }

            if (settings.PrintEvery == 0)
            {
                throw std::invalid_argument("--every must be greater than zero.");
            }

            if (settings.FixedDt <= 0.0f)
            {
                throw std::invalid_argument("--dt must be greater than zero.");
            }

            return settings;
        }

        void PrintFrame(
            const SandboxScene& scene,
            const SandboxFrameStats& stats,
            const SandboxRunSettings& settings)
        {
            std::cout << "\n--- frame " << stats.Step << " ---\n";
            PrintFrameSummary(std::cout, scene, stats);

            if (settings.PrintBodies)
            {
                PrintBodyRows(std::cout, scene);
            }

            if (settings.ShowAscii)
            {
                PrintAsciiSideView(std::cout, scene);
            }
        }
    }

    using namespace detail;

    /// Input: command-line arguments.
    /// Output: process exit code.
    /// Task: drive the terminal sandbox as a real application while keeping the
    /// scenario construction and rendering code reusable for future tools.
    int RunSandboxApp(
        int argc,
        char** argv)
    {
        try
        {
            SandboxRunSettings settings =
                ParseArguments(argc, argv);

            SandboxScene scene =
                MakeDemoScene(settings.Scenario);

            std::ofstream csvFile;
            if (!settings.CsvPath.empty())
            {
                csvFile.open(settings.CsvPath);
                if (!csvFile)
                {
                    throw std::runtime_error(
                        "Failed to open CSV output path '" +
                        settings.CsvPath +
                        "'.");
                }

                WriteCsvHeader(csvFile);
            }

            std::cout
                << "# KairoPhysicsSandbox\n"
                << "# scenario: " << scene.Name << '\n'
                << "# description: " << scene.Description << '\n'
                << "# fixed_dt: " << settings.FixedDt << '\n'
                << "# steps: " << settings.Steps << '\n';

            for (std::uint32_t step = 0; step <= settings.Steps; ++step)
            {
                const float timeSeconds =
                    static_cast<float>(step) * settings.FixedDt;

                const SandboxFrameStats stats =
                    ComputeFrameStats(scene, step, timeSeconds);

                if (step % settings.PrintEvery == 0 || step == settings.Steps)
                {
                    PrintFrame(scene, stats, settings);
                }

                if (csvFile)
                {
                    WriteCsvRow(csvFile, stats);
                }

                if (step < settings.Steps)
                {
                    scene.Projectiles.Step(scene.World, settings.FixedDt);
                    scene.Water.Step(scene.World, settings.FixedDt);
                    scene.World.Step(settings.FixedDt);
                }
            }

            return 0;
        }
        catch (const std::runtime_error& error)
        {
            if (std::string_view(error.what()) == "help requested")
            {
                return 0;
            }

            std::cerr << "KairoPhysicsSandbox error: " << error.what() << '\n';
            return 2;
        }
        catch (const std::exception& error)
        {
            std::cerr << "KairoPhysicsSandbox error: " << error.what() << '\n';
            return 2;
        }
    }
}
