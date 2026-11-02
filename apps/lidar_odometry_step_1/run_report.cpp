#include "run_report.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <pdh.h>
#include <pdhmsg.h>
#include <psapi.h>
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#endif

namespace
{
    std::string utcNow()
    {
        const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm{};
#ifdef _WIN32
        gmtime_s(&tm, &t);
#else
        gmtime_r(&t, &tm);
#endif
        std::ostringstream out;
        out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
        return out.str();
    }

    using Values = std::map<std::string, std::optional<double>>;
    const std::array<const char*, 7> memoryNames = { "process_resident_bytes", "process_private_commit_bytes", "system_available_ram_bytes",
                                                     "system_commit_bytes",    "system_commit_limit_bytes",    "gpu_dedicated_bytes",
                                                     "gpu_shared_bytes" };

    // PDH wildcard counters are collected in the sampling thread only, not per frame or per point.
    // Keep adapter identities in the metric names; summing shared allocations across adapters could double count them.
    class MemorySampler
    {
        std::mutex queryMutex_;
#ifdef _WIN32
        PDH_HQUERY query_ = nullptr;
        PDH_HCOUNTER dedicated_ = nullptr, shared_ = nullptr;
        bool initialized_ = false;
        void gpuCounter(PDH_HCOUNTER counter, const std::string& name, Values& values)
        {
            DWORD bytes = 0, count = 0;
            if (!counter || PdhGetFormattedCounterArrayW(counter, PDH_FMT_LARGE, &bytes, &count, nullptr) != PDH_MORE_DATA)
                return;
            std::vector<unsigned char> buffer(bytes);
            auto* items = reinterpret_cast<PDH_FMT_COUNTERVALUE_ITEM_W*>(buffer.data());
            if (PdhGetFormattedCounterArrayW(counter, PDH_FMT_LARGE, &bytes, &count, items) != ERROR_SUCCESS)
                return;
            double total = 0;
            bool valid = false;
            for (DWORD i = 0; i < count; ++i)
            {
                const auto& value = items[i].FmtValue;
                if (value.CStatus != PDH_CSTATUS_VALID_DATA && value.CStatus != PDH_CSTATUS_NEW_DATA)
                    continue;
                std::wstring instance(items[i].szName);
                const auto luid = instance.find(L"luid_");
                std::string adapter = "unknown";
                if (luid != std::wstring::npos)
                {
                    adapter.clear();
                    for (size_t j = luid; j < instance.size(); ++j)
                        adapter += static_cast<char>(instance[j]); // PDH LUID/physical-index suffix is ASCII
                }
                values[name + "/" + adapter] = static_cast<double>(value.largeValue);
                total += static_cast<double>(value.largeValue);
                valid = true;
            }
            // Totals are convenient for a single GPU; adapter detail remains available in the report.
            if (valid)
                values[name] = total;
        }
#endif
    public:
        ~MemorySampler()
        {
#ifdef _WIN32
            if (query_)
                PdhCloseQuery(query_);
#endif
        }
        Values read()
        {
            std::lock_guard lock(queryMutex_);
            Values values;
            for (const auto* name : memoryNames)
                values[name] = std::nullopt;
#ifdef _WIN32
            PROCESS_MEMORY_COUNTERS_EX process{};
            process.cb = sizeof(process);
            if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&process), sizeof(process)))
            {
                values["process_resident_bytes"] = static_cast<double>(process.WorkingSetSize);
                values["process_private_commit_bytes"] = static_cast<double>(process.PrivateUsage);
            }
            MEMORYSTATUSEX memory{};
            memory.dwLength = sizeof(memory);
            if (GlobalMemoryStatusEx(&memory))
                values["system_available_ram_bytes"] = static_cast<double>(memory.ullAvailPhys);
            PERFORMANCE_INFORMATION performance{};
            performance.cb = sizeof(performance);
            if (GetPerformanceInfo(&performance, sizeof(performance)))
            {
                values["system_commit_bytes"] = static_cast<double>(performance.CommitTotal) * performance.PageSize;
                values["system_commit_limit_bytes"] = static_cast<double>(performance.CommitLimit) * performance.PageSize;
            }
            if (!initialized_)
            {
                initialized_ = true;
                if (PdhOpenQueryW(nullptr, 0, &query_) == ERROR_SUCCESS)
                {
                    const std::wstring prefix = L"\\GPU Process Memory(pid_" + std::to_wstring(GetCurrentProcessId()) + L"_*)\\";
                    PdhAddEnglishCounterW(query_, (prefix + L"Dedicated Usage").c_str(), 0, &dedicated_);
                    PdhAddEnglishCounterW(query_, (prefix + L"Shared Usage").c_str(), 0, &shared_);
                }
            }
            if (query_ && PdhCollectQueryData(query_) == ERROR_SUCCESS)
            {
                gpuCounter(dedicated_, "gpu_dedicated_bytes", values);
                gpuCounter(shared_, "gpu_shared_bytes", values);
            }
#elif defined(__linux__)
            std::ifstream stat("/proc/self/statm");
            std::uint64_t size = 0, resident = 0;
            if (stat >> size >> resident)
                values["process_resident_bytes"] = static_cast<double>(resident) * sysconf(_SC_PAGESIZE);
            std::ifstream info("/proc/meminfo");
            std::string key, rest;
            std::uint64_t kb;
            while (info >> key >> kb)
            {
                std::getline(info, rest);
                if (key == "MemAvailable:")
                    values["system_available_ram_bytes"] = static_cast<double>(kb) * 1024;
            }
#elif defined(__APPLE__)
            mach_task_basic_info_data_t info{};
            mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
            if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, reinterpret_cast<task_info_t>(&info), &count) == KERN_SUCCESS)
                values["process_resident_bytes"] = static_cast<double>(info.resident_size);
#endif
            return values;
        }
    };

    std::shared_ptr<MemorySampler> memorySampler()
    {
        // Process-wide PDH handles: a second run must not repeatedly initialize the GPU performance provider.
        // Each sampler thread retains ownership, so shutdown cannot destroy a backend still in use.
        static auto sampler = std::make_shared<MemorySampler>();
        return sampler;
    }

    std::optional<double> threadCpuSeconds()
    {
#ifdef _WIN32
        FILETIME created{}, exited{}, kernel{}, user{};
        if (GetThreadTimes(GetCurrentThread(), &created, &exited, &kernel, &user))
        {
            ULARGE_INTEGER k{}, u{};
            k.LowPart = kernel.dwLowDateTime;
            k.HighPart = kernel.dwHighDateTime;
            u.LowPart = user.dwLowDateTime;
            u.HighPart = user.dwHighDateTime;
            return static_cast<double>(k.QuadPart + u.QuadPart) / 1e7;
        }
#elif defined(__linux__)
        timespec time{};
        if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time) == 0)
            return time.tv_sec + time.tv_nsec / 1e9;
#endif
        return std::nullopt;
    }

    std::string volumePath(std::filesystem::path path)
    {
        if (path.empty())
            return {};
        std::error_code error;
        while (!path.empty() && !std::filesystem::exists(path, error))
        {
            const auto parent = path.parent_path();
            if (parent == path)
                return {};
            path = parent;
        }
#ifdef _WIN32
        wchar_t root[MAX_PATH + 1]{};
        if (GetVolumePathNameW(std::filesystem::absolute(path, error).c_str(), root, MAX_PATH))
            return std::filesystem::path(root).string();
        return {};
#else
        // Keep a valid path to the filesystem; space() queries the containing mount, not its directory contents.
        return std::filesystem::absolute(path, error).string();
#endif
    }

#ifdef _WIN32
    BOOL CALLBACK pagefileVolume(LPVOID context, PENUM_PAGE_FILE_INFORMATION, LPCWSTR name)
    {
        auto* report = static_cast<RunReport*>(context);
        std::wstring path(name);
        if (path.rfind(L"\\??\\", 0) == 0)
            path.erase(0, 4);
        report->addVolume(std::filesystem::path(path).parent_path(), "pagefile");
        return TRUE;
    }
#endif
} // namespace

void RunReport::Metric::observe(std::optional<double> value)
{
    current = value;
    if (!value)
        return;
    if (!first)
        first = value;
    minimum = minimum ? std::min(*minimum, *value) : value;
    maximum = maximum ? std::max(*maximum, *value) : value;
    ++samples;
}

void RunReport::Metric::integrate(double seconds)
{
    if (current)
    {
        integral += *current * seconds;
        covered += seconds;
    }
}

nlohmann::json RunReport::Metric::json() const
{
    auto nullable = [](std::optional<double> value)
    {
        return value ? nlohmann::json(*value) : nlohmann::json(nullptr);
    };
    return { { "first_bytes", nullable(first) },
             { "last_bytes", nullable(current) },
             { "min_bytes", nullable(minimum) },
             { "max_sampled_bytes", nullable(maximum) },
             { "time_weighted_mean_bytes", covered > 0 ? nlohmann::json(integral / covered) : nlohmann::json(nullptr) },
             { "coverage_seconds", covered },
             { "sample_count", samples },
             { "availability", samples ? "observed" : "unsupported_or_query_failed" } };
}

const char* RunReport::stageName(Stage stage)
{
    static const char* names[] = {
        "loading", "initialization", "cache_preparation", "odometry", "export", "cleanup", "consistency", "idle"
    };
    return names[static_cast<size_t>(stage)];
}

const char* RunReport::stageLabel(Stage stage)
{
    static const char* labels[] = { "Loading data",    "Initial trajectory", "Preparing data and cache", "Odometry",
                                    "Writing results", "Removing cache",     "Trajectory consistency",   "Ready" };
    return labels[static_cast<size_t>(stage)];
}

RunReport::Scope::Scope(RunReport* report, Stage stage)
    : report(report)
    , exceptions(std::uncaught_exceptions())
{
    if (report)
        report->begin(stage);
}
RunReport::Scope::~Scope()
{
    if (report && std::uncaught_exceptions() > exceptions)
        report->fail("Exception during measured stage");
    finish();
}
void RunReport::Scope::finish()
{
    if (report)
        report->end();
    report = nullptr;
}

RunReport::RunReport(const std::filesystem::path& input, const std::filesystem::path& spill, const std::string& mode)
    : mode_(mode)
{
    started_ = updated_ = operationStarted_ = utcNow();
    id_ = started_ + "-" + std::to_string(Clock::now().time_since_epoch().count());
    addVolume(input, "input/cache");
    addVolume(spill, "spill");
#ifdef _WIN32
    EnumPageFilesW(pagefileVolume, this);
#endif
    setupCost_ = std::chrono::duration<double>(Clock::now() - last_).count();
    sampler_ = std::thread(&RunReport::sampleLoop, this);
}
RunReport::~RunReport()
{
    {
        std::lock_guard lock(mutex_);
        stopping_ = true;
    }
    wake_.notify_all();
    if (sampler_.joinable())
        sampler_.join();
}

void RunReport::account(Clock::time_point now)
{
    const double seconds = std::chrono::duration<double>(now - last_).count();
    last_ = now;
    if (!active_)
        return;
    total_ += seconds;
    if (paused_)
        pausedSeconds_ += seconds;
    for (auto& [name, metric] : memory_)
        metric.integrate(seconds);
    for (auto& volume : volumes_)
        volume.free.integrate(seconds);
    if (stage_ != Stage::Count)
    {
        auto& stage = stages_[static_cast<size_t>(stage_)];
        stage.seconds += seconds;
        if (paused_)
            stage.paused += seconds;
        for (auto& [name, metric] : stage.memory)
            metric.integrate(seconds);
        for (auto& [path, metric] : stage.disk)
            metric.integrate(seconds);
    }
}

void RunReport::begin(Stage stage)
{
    std::lock_guard lock(mutex_);
    account(Clock::now());
    if (!active_)
    {
        operationStarted_ = utcNow();
        operationStartTotal_ = total_;
        operationStartPaused_ = pausedSeconds_;
        // Never extrapolate the last reading across an unmeasured interactive interval.
        for (auto& [name, metric] : memory_)
            metric.current.reset();
        for (auto& volume : volumes_)
            volume.free.current.reset();
    }
    active_ = true;
    status_ = "running";
    stage_ = stage;
    auto& data = stages_[static_cast<size_t>(stage)];
    ++data.calls;
    for (auto& [name, metric] : memory_)
        data.memory[name].observe(metric.current);
    for (const auto& volume : volumes_)
        data.disk[volume.path].observe(volume.free.current);
    sampleRequested_ = true;
    wake_.notify_one();
}
void RunReport::end()
{
    std::lock_guard lock(mutex_);
    account(Clock::now());
    stage_ = Stage::Count;
    sampleRequested_ = true;
    wake_.notify_one();
}
void RunReport::pause(bool paused)
{
    std::lock_guard lock(mutex_);
    account(Clock::now());
    paused_ = paused;
}
void RunReport::finishOperation(const std::string& status)
{
    std::lock_guard lock(mutex_);
    account(Clock::now());
    if (status == "failed" || status == "cancelled")
        outcome_ = status;
    else if (outcome_ != "failed" && outcome_ != "cancelled")
        outcome_ = errors_.empty() ? status : "partial_failure";
    if (active_)
        operations_.push_back(
            { { "started_utc", operationStarted_ },
              { "finished_utc", utcNow() },
              { "wall_seconds", total_ - operationStartTotal_ },
              { "paused_seconds", pausedSeconds_ - operationStartPaused_ },
              { "status", outcome_ } });
    active_ = false;
    stage_ = Stage::Count;
    paused_ = false;
    status_ = outcome_;
    updated_ = utcNow();
    wake_.notify_one();
}
void RunReport::fail(const std::string& error)
{
    std::lock_guard lock(mutex_);
    errors_.push_back(error);
    status_ = "partial_failure";
}

void RunReport::addVolume(const std::filesystem::path& path, const std::string& role)
{
    const auto root = volumePath(path);
    if (root.empty())
        return;
    std::lock_guard lock(mutex_);
    auto it = std::find_if(
        volumes_.begin(),
        volumes_.end(),
        [&](const Volume& v)
        {
            return v.path == root;
        });
    if (it == volumes_.end())
    {
        volumes_.push_back({ root, { role }, {} });
    }
    else if (std::find(it->roles.begin(), it->roles.end(), role) == it->roles.end())
        it->roles.push_back(role);
}

bool RunReport::cacheFile(
    const std::filesystem::path& path, double writeSeconds, const std::string& kind, std::optional<std::uint64_t> expectedBytes)
{
    const auto start = Clock::now();
    std::error_code error;
    const auto bytes = std::filesystem::file_size(path, error); // once per completed file; no directory traversal
    std::lock_guard lock(mutex_);
    cacheWriteSeconds_ += writeSeconds;
    cacheWriteByKind_[kind] += writeSeconds;
    if (error || (expectedBytes && bytes != *expectedBytes))
    {
        errors_.push_back("Missing or incomplete cache file: " + path.string());
        cacheAccountingCost_ += std::chrono::duration<double>(Clock::now() - start).count();
        return false;
    }
    auto& entry = cacheFiles_[path.string()];
    cacheBytes_ -= entry.first;
    entry = { bytes, kind };
    cacheBytes_ += bytes;
    cachePeak_ = std::max(cachePeak_, cacheBytes_);
    cachePeakFiles_ = std::max(cachePeakFiles_, static_cast<std::uint64_t>(cacheFiles_.size()));
    cacheStatus_ = "retained";
    cacheAccountingCost_ += std::chrono::duration<double>(Clock::now() - start).count();
    return true;
}
void RunReport::cachePrepared()
{
    std::lock_guard lock(mutex_);
    preparedBytes_ = cacheBytes_;
    preparedFiles_ = cacheFiles_.size();
}
void RunReport::cacheRemoved(bool success, const std::string& error)
{
    std::lock_guard lock(mutex_);
    cacheStatus_ = success ? "removed" : "cleanup_failed";
    if (success)
    {
        cacheBytes_ = 0;
        cacheFiles_.clear();
    }
    else
        errors_.push_back("Cache cleanup: " + error);
}
void RunReport::spillSize(std::uint64_t bytes, bool removed)
{
    std::lock_guard lock(mutex_);
    spillBytes_ = removed ? 0 : bytes;
    spillPeak_ = std::max(spillPeak_, bytes);
}

void RunReport::sampleLoop()
{
    const auto sampler = memorySampler();
    const auto cpuStart = threadCpuSeconds();
    auto nextDisk = Clock::time_point{};
    std::unique_lock lock(mutex_);
    while (!stopping_)
    {
        if (!active_)
        {
            wake_.wait(
                lock,
                [&]
                {
                    return stopping_ || active_;
                });
            continue;
        }
        sampleRequested_ = false;
        std::vector<std::string> paths;
        const bool disk = Clock::now() >= nextDisk;
        if (disk)
            for (const auto& volume : volumes_)
                paths.push_back(volume.path);
        lock.unlock();
        const auto start = Clock::now();
        Values values;
        try
        {
            values = sampler->read();
        } catch (const std::exception&)
        {
        } // telemetry must not terminate a processing run
        std::map<std::string, std::optional<double>> free;
        for (const auto& path : paths)
        {
            std::error_code error;
            auto space = std::filesystem::space(path, error);
            free[path] = error ? std::nullopt : std::optional<double>(static_cast<double>(space.available));
        }
        const auto now = Clock::now();
        const double cost = std::chrono::duration<double>(now - start).count();
        const auto cpuNow = threadCpuSeconds();
        lock.lock();
        account(Clock::now()); // another stage transition may have accounted time while the OS query was in flight
        if (!active_)
            continue; // final snapshots remain immutable until the next explicit operation
        sampleCost_ += cost;
        if (!sampleCount_)
            firstSampleCost_ = cost;
        if (cpuStart && cpuNow)
            samplerCpuSeconds_ = *cpuNow - *cpuStart;
        maxSampleCost_ = std::max(maxSampleCost_, cost);
        ++sampleCount_;
        for (auto& [name, metric] : memory_)
            if (!values.count(name))
                metric.observe(std::nullopt);
        for (const auto& [name, value] : values)
        {
            memory_[name].observe(value);
            if (active_ && stage_ != Stage::Count)
                stages_[static_cast<size_t>(stage_)].memory[name].observe(value);
        }
        for (auto& volume : volumes_)
            if (free.count(volume.path))
            {
                volume.free.observe(free[volume.path]);
                if (stage_ != Stage::Count)
                    stages_[static_cast<size_t>(stage_)].disk[volume.path].observe(free[volume.path]);
            }
        if (disk)
            nextDisk = now + std::chrono::seconds(5);
        // Bound even stage-boundary bursts; OS calls never run under the report mutex.
        wake_.wait_until(
            lock,
            now + std::chrono::milliseconds(100),
            [&]
            {
                return stopping_;
            });
        wake_.wait_until(
            lock,
            now + std::chrono::seconds(1),
            [&]
            {
                return stopping_ || !active_ || sampleRequested_;
            });
    }
}

double RunReport::stageSeconds(Stage stage, bool excludePauses) const
{
    std::lock_guard lock(mutex_);
    const auto& data = stages_[static_cast<size_t>(stage)];
    const double pending = active_ && stage_ == stage ? std::chrono::duration<double>(Clock::now() - last_).count() : 0;
    return data.seconds - (excludePauses ? data.paused : 0) + (excludePauses && paused_ ? 0 : pending);
}
void RunReport::setRecordingSpan(double firstTimestamp, double lastTimestamp)
{
    std::lock_guard lock(mutex_);
    recordingFirst_.reset();
    recordingLast_.reset();
    if (std::isfinite(firstTimestamp) && std::isfinite(lastTimestamp) && lastTimestamp >= firstTimestamp &&
        std::isfinite(lastTimestamp - firstTimestamp))
    {
        recordingFirst_ = firstTimestamp;
        recordingLast_ = lastTimestamp;
    }
}

std::optional<double> RunReport::recordingSeconds() const
{
    std::lock_guard lock(mutex_);
    return recordingFirst_ && recordingLast_ ? std::optional<double>(*recordingLast_ - *recordingFirst_) : std::nullopt;
}

RunReport::Live RunReport::live() const
{
    std::lock_guard lock(mutex_);
    const double pending = active_ ? std::chrono::duration<double>(Clock::now() - last_).count() : 0;
    return { stageName(stage_), stageLabel(stage_), status_,     total_ + pending, pausedSeconds_ + (paused_ ? pending : 0),
             memory_,           volumes_,           cacheBytes_, cachePeak_ };
}

nlohmann::json RunReport::jsonLocked() const
{
    nlohmann::json stages = nlohmann::json::object(), resources = nlohmann::json::object();
    double stageTotal = 0;
    for (size_t i = 0; i < count; ++i)
    {
        const auto& data = stages_[i];
        stageTotal += data.seconds;
        auto& stage = stages[stageName(static_cast<Stage>(i))];
        stage = { { "wall_seconds", data.seconds },
                  { "paused_seconds", data.paused },
                  { "excluding_pauses_seconds", data.seconds - data.paused },
                  { "invocations", data.calls } };
        for (const auto& [name, metric] : data.memory)
            stage["memory"][name] = metric.json();
        for (const auto& [path, metric] : data.disk)
            stage["available_disk_space"][path] = metric.json();
    }
    for (const auto& [name, metric] : memory_)
        resources[name] = metric.json();
    nlohmann::json volumes = nlohmann::json::array();
    for (const auto& volume : volumes_)
        volumes.push_back({ { "path", volume.path }, { "roles", volume.roles }, { "available_space", volume.free.json() } });
    return {
        { "schema_version", 1 },
        { "run_id", id_ },
        { "started_utc", started_ },
        { "updated_utc", updated_ },
        { "status", status_ },
        { "mode", mode_ },
        { "errors", errors_ },
        { "operations", operations_ },
        { "input_recording",
          { { "source", "loaded_raw_imu_timestamps" },
            { "start_timestamp_seconds", recordingFirst_ ? nlohmann::json(*recordingFirst_) : nlohmann::json(nullptr) },
            { "end_timestamp_seconds", recordingLast_ ? nlohmann::json(*recordingLast_) : nlohmann::json(nullptr) },
            { "span_seconds",
              recordingFirst_ && recordingLast_ ? nlohmann::json(*recordingLast_ - *recordingFirst_) : nlohmann::json(nullptr) },
            { "includes_gaps", true } } },
        { "timing",
          { { "total_operation_wall_seconds", total_ },
            { "paused_seconds", pausedSeconds_ },
            { "excluding_pauses_seconds", total_ - pausedSeconds_ },
            { "between_stages_seconds", std::max(0.0, total_ - stageTotal) },
            { "scope",
              "Cumulative operation intervals; includes pauses, excludes interactive idle between operations and final report "
              "serialization" },
            { "stages", stages },
            { "cache_write_seconds_nested", cacheWriteSeconds_ },
            { "cache_write_seconds_by_kind_nested", cacheWriteByKind_ } } },
        { "memory", resources },
        { "volumes", volumes },
        { "disk_cache",
          { { "status", cacheStatus_ },
            { "scope", "Completed cache-file writes tracked for this run; unmodified pre-existing files are excluded" },
            { "prepared_bytes", preparedBytes_ },
            { "prepared_file_count", preparedFiles_ },
            { "current_bytes", cacheStatus_ == "cleanup_failed" ? nlohmann::json(nullptr) : nlohmann::json(cacheBytes_) },
            { "peak_bytes", cachePeak_ },
            { "current_file_count", cacheStatus_ == "cleanup_failed" ? nlohmann::json(nullptr) : nlohmann::json(cacheFiles_.size()) },
            { "peak_file_count", cachePeakFiles_ },
            { "spill_current_bytes", spillBytes_ },
            { "spill_scope",
              "Current stream; peak includes observed generation overlap. Cleanup failures may leave other files; see errors" },
            { "spill_peak_bytes", spillPeak_ } } },
        { "sampling",
          { { "interval_seconds", 1 },
            { "disk_interval_seconds", 5 },
            { "sample_count", sampleCount_ },
            { "query_wall_seconds", sampleCost_ },
            { "max_query_wall_seconds", maxSampleCost_ },
            { "first_query_wall_seconds", firstSampleCost_ },
            { "subsequent_query_wall_seconds", sampleCost_ - firstSampleCost_ },
            { "sampler_thread_cpu_seconds", samplerCpuSeconds_ ? nlohmann::json(*samplerCpuSeconds_) : nlohmann::json(nullptr) },
            { "setup_wall_seconds", setupCost_ },
            { "cache_accounting_wall_seconds", cacheAccountingCost_ },
            { "query_wall_fraction", total_ > 0 ? sampleCost_ / total_ : 0 },
            { "average_method", "Time-weighted last observation, over valid coverage including pauses" },
            { "stage_boundaries", "Carry the most recent available sample within an operation; refresh asynchronously" },
            { "peak_method", "Sampled within this run; short spikes may be missed" },
            { "source", "Native OS process/system memory APIs; Windows GPU Process Memory PDH counters (driver/OS limitations apply)" },
            { "commit_definition", "Private committed memory, not physical pagefile occupancy" } } }
    };
}
nlohmann::json RunReport::json() const
{
    std::lock_guard lock(mutex_);
    return jsonLocked();
}
std::string RunReport::summary() const
{
    const auto report = json();
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << "\nTotal operations: " << report["timing"]["total_operation_wall_seconds"].get<double>()
        << " s (paused: " << report["timing"]["paused_seconds"].get<double>() << " s)\n";
    for (size_t i = 0; i < count; ++i)
    {
        const char* name = stageName(static_cast<Stage>(i));
        const auto& stage = report["timing"]["stages"][name];
        if (stage["invocations"].get<std::uint64_t>())
            out << stageLabel(static_cast<Stage>(i)) << ": " << stage["wall_seconds"].get<double>() << " s\n";
    }
    out << "Cache writes (included above): " << report["timing"]["cache_write_seconds_nested"].get<double>() << " s\n";
    for (const auto& [name, label] :
         std::array<std::pair<const char*, const char*>, 4>{ { { "process_resident_bytes", "Application resident RAM" },
                                                               { "process_private_commit_bytes", "Application private commit" },
                                                               { "gpu_dedicated_bytes", "GPU dedicated (adapter sum)" },
                                                               { "gpu_shared_bytes", "GPU shared (adapter sum)" } } })
    {
        out << label << " peak sampled: ";
        if (report["memory"].contains(name) && !report["memory"][name]["max_sampled_bytes"].is_null())
            out << report["memory"][name]["max_sampled_bytes"].get<double>() / (1024 * 1024) << " MiB\n";
        else
            out << "N/A\n";
    }
    out << "Cache: " << report["disk_cache"]["status"].get<std::string>() << "; peak "
        << static_cast<double>(report["disk_cache"]["peak_bytes"].get<std::uint64_t>()) / (1024 * 1024) << " MiB\n"
        << "Status: " << report["status"].get<std::string>() << "\n";
    return out.str();
}

bool RunReport::persist(const std::filesystem::path& path, nlohmann::json metadata)
{
    metadata["run_analysis"] = json();
    const auto temporary = std::filesystem::path(path.string() + ".tmp");
    {
        std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
        out << metadata.dump(4) << '\n';
        out.close();
        if (!out)
            return false;
    }
#ifdef _WIN32
    return MoveFileExW(temporary.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING) != 0;
#else
    std::error_code error;
    std::filesystem::rename(temporary, path, error);
    return !error;
#endif
}
bool RunReport::write(const std::filesystem::path& path, nlohmann::json metadata)
{
    std::lock_guard lock(filesMutex_);
    reports_[path] = metadata;
    return persist(path, std::move(metadata));
}
bool RunReport::updateReports()
{
    std::lock_guard lock(filesMutex_);
    bool success = true;
    for (const auto& [path, metadata] : reports_)
        success = persist(path, metadata) && success;
    return success;
}
