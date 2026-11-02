#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// Wall-clock accounting only: stage durations include I/O and rendering contention, not summed CPU time.
// One pipeline producer changes stages; the sampler never reads pipeline data or scans cache directories.
class RunReport
{
public:
    using Clock = std::chrono::steady_clock;
    enum class Stage
    {
        Loading,
        Initialization,
        Cache,
        Odometry,
        Export,
        Cleanup,
        Consistency,
        Count
    };
    static const char* stageName(Stage stage);
    static const char* stageLabel(Stage stage);
    struct Metric
    {
        std::optional<double> current, minimum, maximum, first;
        double integral = 0, covered = 0;
        std::uint64_t samples = 0;
        void observe(std::optional<double> value);
        void integrate(double seconds);
        nlohmann::json json() const;
    };
    struct Volume
    {
        std::string path;
        std::vector<std::string> roles;
        Metric free;
    };
    struct Live
    {
        std::string stage, label, status;
        double total = 0, paused = 0;
        std::map<std::string, Metric> memory;
        std::vector<Volume> volumes;
        std::uint64_t cacheBytes = 0, cachePeak = 0;
    };
    struct Scope
    {
        RunReport* report;
        int exceptions;
        explicit Scope(RunReport* report, Stage stage);
        ~Scope();
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;
        void finish();
    };

    explicit RunReport(const std::filesystem::path& input, const std::filesystem::path& spill = {}, const std::string& mode = "library");
    ~RunReport();
    RunReport(const RunReport&) = delete;
    RunReport& operator=(const RunReport&) = delete;
    void begin(Stage stage);
    void end();
    void pause(bool paused);
    void finishOperation(const std::string& status = "completed");
    void fail(const std::string& error);
    bool cacheFile(
        const std::filesystem::path& path, double writeSeconds, const std::string& kind, std::optional<std::uint64_t> expectedBytes = {});
    void cachePrepared();
    void cacheRemoved(bool success, const std::string& error);
    void spillSize(std::uint64_t bytes, bool removed = false);
    void addVolume(const std::filesystem::path& path, const std::string& role);
    double stageSeconds(Stage stage, bool excludePauses = false) const;
    void setRecordingSpan(double firstTimestamp, double lastTimestamp);
    std::optional<double> recordingSeconds() const;
    Live live() const;
    nlohmann::json json() const;
    std::string summary() const;
    // The final report's serialization/replace is explicitly outside measured operation totals.
    bool write(const std::filesystem::path& path, nlohmann::json metadata);
    bool updateReports();

private:
    static constexpr size_t count = static_cast<size_t>(Stage::Count);
    struct StageData
    {
        double seconds = 0, paused = 0;
        std::uint64_t calls = 0;
        std::map<std::string, Metric> memory;
        std::map<std::string, Metric> disk;
    };
    void account(Clock::time_point now);
    void sampleLoop();
    nlohmann::json jsonLocked() const;
    bool persist(const std::filesystem::path& path, nlohmann::json metadata);
    mutable std::mutex mutex_;
    std::mutex filesMutex_;
    std::condition_variable wake_;
    std::thread sampler_;
    bool stopping_ = false, active_ = true, paused_ = false, sampleRequested_ = true;
    Stage stage_ = Stage::Count;
    Clock::time_point last_ = Clock::now();
    double total_ = 0, pausedSeconds_ = 0, sampleCost_ = 0, maxSampleCost_ = 0, setupCost_ = 0, cacheAccountingCost_ = 0;
    double firstSampleCost_ = 0;
    std::optional<double> samplerCpuSeconds_;
    double operationStartTotal_ = 0, operationStartPaused_ = 0;
    std::uint64_t sampleCount_ = 0;
    std::string id_, started_, updated_, status_ = "running", cacheStatus_ = "not_created";
    std::string operationStarted_, outcome_ = "completed";
    std::string mode_;
    std::optional<double> recordingFirst_, recordingLast_;
    nlohmann::json operations_ = nlohmann::json::array();
    std::vector<std::string> errors_;
    std::array<StageData, count> stages_;
    std::map<std::string, Metric> memory_;
    std::vector<Volume> volumes_;
    std::map<std::string, std::pair<std::uint64_t, std::string>> cacheFiles_;
    std::uint64_t cacheBytes_ = 0, cachePeak_ = 0, cachePeakFiles_ = 0, preparedBytes_ = 0, preparedFiles_ = 0;
    std::uint64_t spillBytes_ = 0, spillPeak_ = 0;
    double cacheWriteSeconds_ = 0;
    std::map<std::string, double> cacheWriteByKind_;
    std::map<std::filesystem::path, nlohmann::json> reports_;
};
