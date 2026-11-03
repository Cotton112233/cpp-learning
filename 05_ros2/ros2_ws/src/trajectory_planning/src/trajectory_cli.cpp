#include <charconv>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <string>
#include <string_view>
#include <system_error>

#include "trajectory_planning/step_linear.hpp"
#include "trajectory_planning/polynomial.hpp"

namespace
{

bool parseDouble(std::string_view text, double& value)
{
    const char* begin = text.data();
    const char* end = begin + text.size();
    auto result = std::from_chars(begin, end, value);
    return result.ec == std::errc{} && result.ptr == end && std::isfinite(value);
}

int error(const std::string& message)
{
    std::cerr << "Error: " << message << '\n';
    return 2;
}

void printHelp()
{
    std::cout
        << "Usage: trajectory_cli [--algorithm linear|step|cubic|quintic]"
           " [--q0 RAD] [--qf RAD] [--T SECONDS] [--dt SECONDS]\n"
        << "Defaults: --algorithm linear --q0 0 --qf 1 --T 4 --dt 0.1\n"
        << "Successful runs write only CSV: time,position,velocity,acceleration.\n"
        << "Samples include -dt, 0, interior grid points, exact T, and T+dt.\n"
        << "Step switches at t=0; T is its observation window.\n"
        << "nan marks undefined derivatives: step/linear boundaries or cubic endpoint acceleration.\n"
        << "T and dt must be positive; at most 1000000 motion intervals.\n";
}

}  // namespace

int main(int argc, char* argv[])
{
    trajectory_planning::TrajectoryParameters parameters;
    std::string algorithm = "linear";
    double dt = 0.1;

    for (int i = 1; i < argc; ++i)
    {
        std::string option = argv[i];
        if (option == "--help" || option == "-h")
        {
            printHelp();
            return 0;
        }
        if (option != "--algorithm" && option != "--q0" && option != "--qf" &&
            option != "--T" && option != "--dt")
        {
            return error("unknown option: " + option);
        }
        if (i + 1 >= argc)
        {
            return error("missing value for " + option);
        }
        std::string value_text = argv[++i];
        if (option == "--algorithm")
        {
            algorithm = value_text;
            continue;
        }

        double value = 0.0;
        if (!parseDouble(value_text, value))
        {
            return error("expected a finite number for " + option + ": " + value_text);
        }
        if (option == "--q0") parameters.q0 = value;
        else if (option == "--qf") parameters.qf = value;
        else if (option == "--T") parameters.duration = value;
        else if (option == "--dt") dt = value;
    }

    if (algorithm != "linear" && algorithm != "step" &&
        algorithm != "cubic" && algorithm != "quintic")
    {
        return error("--algorithm must be linear, step, cubic or quintic");
    }
    if (parameters.duration <= 0.0 || dt <= 0.0)
    {
        return error("--T and --dt must be > 0");
    }

    double intervals = std::ceil(parameters.duration / dt);
    if (!std::isfinite(intervals) || intervals > 1000000.0)
    {
        return error("too many samples; increase --dt or decrease --T");
    }
    // Even if T / dt underflows, include the t=0 sample.
    if (intervals < 1.0)
    {
        intervals = 1.0;
    }
    if (!std::isfinite(parameters.duration + dt))
    {
        return error("T + dt is outside the representable time range");
    }
    if (algorithm != "step" &&
        (!std::isfinite(parameters.qf - parameters.q0) ||
         !std::isfinite((parameters.qf - parameters.q0) / parameters.duration)))
    {
        return error("trajectory displacement or velocity is outside the numeric range");
    }

    std::cout.imbue(std::locale::classic());
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    std::cout << "time,position,velocity,acceleration\n";

    auto printSample = [&](double time)
    {
        trajectory_planning::TrajectoryState state;
        if (algorithm == "linear")
            state = trajectory_planning::linearTrajectory(time, parameters);
        else if (algorithm == "step")
            state = trajectory_planning::stepTrajectory(time, parameters);
        else if (algorithm == "cubic")
            state = trajectory_planning::cubicTrajectory(time, parameters);
        else
            state = trajectory_planning::quinticTrajectory(time, parameters);
        std::cout << time << ',' << state.position << ','
                  << state.velocity << ',' << state.acceleration << '\n';
    };

    printSample(-dt);
    // Compute each sample from its index, avoiding accumulated time += dt error.
    for (std::size_t i = 0; i < static_cast<std::size_t>(intervals); ++i)
    {
        double time = static_cast<double>(i) * dt;
        if (time < parameters.duration)
        {
            printSample(time);
        }
    }
    // Always include the exact endpoint, even when T is not divisible by dt.
    printSample(parameters.duration);
    printSample(parameters.duration + dt);
    return std::cout ? 0 : 1;
}
