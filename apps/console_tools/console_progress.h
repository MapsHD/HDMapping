#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

/**
 * @brief Progress of one long-running step of a console tool, reported through spdlog.
 *
 * Plain log lines rather than a redrawn bar, so it reads the same in a terminal, a pipe and a CI log,
 * and interleaves cleanly with the tools' other messages.
 */
class ConsoleProgress
{
public:
    /**
     * @brief Starts the clock of a step.
     * @param label Name of the step, printed at the start of each line.
     * @param total Number of units of work, or 0 if unknown (only the count is reported then).
     */
    ConsoleProgress(std::string label, uint64_t total)
        : label_(std::move(label))
        , total_(total)
        , start_(Clock::now())
        , last_report_(start_)
    {
    }

    /**
     * @brief Logs the progress, at most once per reporting interval. Cheap enough to call per point.
     * @param done Units of work finished so far.
     */
    void update(uint64_t done)
    {
        const auto now = Clock::now();
        if (now - last_report_ < kReportInterval)
            return;
        last_report_ = now;
        spdlog::info("{}: {}", label_, status(done));
    }

    /**
     * @brief Logs the closing line of the step. The count may fall short of the total: messages skipped, read cut short.
     * @param done Units of work finished.
     */
    void finish(uint64_t done) const
    {
        spdlog::info("{}: done, {} in {}", label_, done, formatSeconds(elapsedSeconds()));
    }

    /**
     * @brief Describes the progress, for tools that already log a line per unit of work.
     * @param done Units of work finished so far.
     * @return For example "42%, 1200/3000, 1m 20s left", or only the count if the total is unknown.
     */
    std::string status(uint64_t done) const
    {
        if (total_ == 0)
            return std::to_string(done);

        const uint64_t clamped = std::min(done, total_);
        const double fraction = static_cast<double>(clamped) / static_cast<double>(total_);
        std::string text = fmt::format("{:.0f}%, {}/{}", fraction * 100.0, clamped, total_);
        if (clamped > 0 && clamped < total_)
            text += ", " + formatSeconds(elapsedSeconds() * (1.0 - fraction) / fraction) + " left";
        return text;
    }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr std::chrono::seconds kReportInterval{ 2 };

    double elapsedSeconds() const
    {
        return std::chrono::duration<double>(Clock::now() - start_).count();
    }

    static std::string formatSeconds(double seconds)
    {
        const auto whole = static_cast<uint64_t>(seconds + 0.5);
        if (whole < 60)
            return fmt::format("{}s", whole);
        if (whole < 3600)
            return fmt::format("{}m {}s", whole / 60, whole % 60);
        return fmt::format("{}h {}m", whole / 3600, (whole % 3600) / 60);
    }

    std::string label_;
    uint64_t total_;
    Clock::time_point start_;
    Clock::time_point last_report_;
};
