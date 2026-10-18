#pragma once

#include "pipeline.h"
#include "preflight.h"
#include "settings.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// Batch mode: several recordings, one after another, through the unchanged single-recording Pipeline. Each recording
// gets its own output folder <output root>/<output name>/ with the usual result files; the output root also holds
// batch.log and batch_summary.csv. See docs/DESIGN.md, "Batch mode".
namespace easy_gui
{
    //! A folder found by discoverRecordings().
    struct DiscoveredFolder
    {
        std::filesystem::path path;
        std::string name; // folder name (the parent's own name when the parent itself is a recording)
        bool is_recording = false;
        std::string skip_reason; // plain language, when !is_recording
    };

    //! The recordings among `parent`'s direct subfolders, plus `parent` itself if it is a recording (listed first).
    //! Not recursive: a recording is the folder the scanner wrote, and searching deeper would also find step 1's own
    //! folders in old manual runs. A recording is a folder with at least one point cloud and one IMU file as
    //! classifyRecordingFiles sees them; the pre-flight check judges the rest. Subfolders that are not recordings are
    //! listed with the reason; hidden and system folders (".*", "$*", "System Volume Information") are left out.
    std::vector<DiscoveredFolder> discoverRecordings(const std::filesystem::path& parent);

    //! Why `name` cannot be used as an output folder name, or empty if it can. It must be a single folder name that
    //! is valid on both Windows and macOS.
    std::string outputNameProblem(const std::string& name);

    //! Output names are compared ignoring (ASCII) case: the default file systems of macOS and Windows do.
    bool sameOutputName(const std::string& a, const std::string& b);

    //! Identifies a recording independently of where it is mounted (a USB stick may get another drive letter):
    //! number of file pairs, the total size of the paired files and the first IMU timestamp. Reads file sizes and the
    //! first lines of one IMU file only.
    std::string recordingFingerprint(const RecordingFiles& files);

    //! Default output names: the folder names (characters invalid in a folder name replaced by '_'), made unique
    //! ignoring case with _2, _3, ... A name is also passed over when its folder under `output_root` already holds
    //! something other than this recording's finished result or an unfinished attempt.
    std::vector<std::string> defaultOutputNames(const std::vector<DiscoveredFolder>& recordings, const std::filesystem::path& output_root);

    //! Contents of easy_gui_done.txt, the marker batch mode writes into a recording's output folder once its result
    //! is complete. A restarted batch skips recordings whose marker matches.
    struct DoneMarker
    {
        std::string recording; // path at the time it was processed (informative)
        std::string fingerprint; // recordingFingerprint(): what identifies the recording
        std::string finished; // local time
        double duration_s = 0.0;
        uint64_t raw_points = 0;
        uint64_t exported_points = 0;
    };

    bool writeDoneMarker(const std::filesystem::path& output_folder, const DoneMarker& marker, std::string& error);
    std::optional<DoneMarker> readDoneMarker(const std::filesystem::path& output_folder);

    //! Disk space a batch needs at its start: the recordings run one at a time and each frees its temporary files
    //! when it ends, so only the largest temporary need counts, plus the results of all of them.
    struct BatchDiskNeed
    {
        uint64_t work_bytes = 0;
        uint64_t result_bytes = 0;
        uint64_t total() const
        {
            return work_bytes + result_bytes;
        }
    };

    BatchDiskNeed batchDiskNeed(const std::vector<const PreflightResult*>& recordings);

    //! The batch disk finding, with the same thresholds as the single-recording check (error below the estimate,
    //! warning below 1.5x). `available` is ignored when `space_known` is false.
    Finding batchDiskFinding(const BatchDiskNeed& need, bool space_known, uint64_t available);

    //! Why the batch cannot write into `output_root`, or empty: batch.log and batch_summary.csv go there, so it must
    //! not be inside any recording (the per-recording check catches the recordings' own output folders).
    std::string batchOutputProblem(const std::filesystem::path& output_root, const std::vector<DiscoveredFolder>& folders);

    //! Pre-flight result of one batch recording, as shown in the table.
    struct BatchCheck
    {
        PreflightResult preflight; // checked with resume_unfinished, against <output root>/<output name>
        std::string fingerprint;
        std::optional<DoneMarker> done; // set if the output folder already holds this recording's finished result
    };

    BatchCheck checkBatchRecording(const std::filesystem::path& recording, const std::filesystem::path& output_folder);

    //! First error of a pre-flight result (empty if none).
    std::string firstError(const PreflightResult& result);

    // ------------------------------------------------------------------------------------------------------------
    // Running

    enum class BatchItemState
    {
        Excluded, // not selected, or the check failed
        Pending,
        Running,
        Succeeded,
        Failed,
        Cancelled, // stopped by "stop now" while it ran
        AlreadyDone, // finished in an earlier run of the same batch
        NotRun, // the batch was stopped before it got there
    };

    //! Token in batch_summary.csv ("succeeded", "already_done", ...).
    const char* batchStateToken(BatchItemState state);

    struct BatchItem
    {
        std::filesystem::path recording;
        std::string name; // as discovered (shown and logged)
        std::string output_name;
        bool include = true;
        std::string exclude_reason; // written to the summary when !include
        uint64_t total_points = 0; // from the check; weights the overall progress (0 = unknown, use the average)
    };

    struct BatchRequest
    {
        std::filesystem::path output_root;
        std::vector<BatchItem> items;
        std::vector<DiscoveredFolder> skipped; // folders that are not recordings: only for the summary and the log
        std::filesystem::path step1_executable;
        Settings settings;
        double check_band_outer_m = 0.0; // dev/test option, passed to every recording
    };

    struct BatchItemResult
    {
        BatchItemState state = BatchItemState::Pending;
        double duration_s = 0.0;
        uint64_t raw_points = 0;
        uint64_t exported_points = 0;
        std::string error;
        std::filesystem::path output_folder;
    };

    struct BatchStatus
    {
        bool running = false;
        std::vector<BatchItemResult> items; // parallel to BatchRequest::items
        int current = -1; // index of the item being processed
        PipelineStatus current_status; // of the item being processed
        double elapsed_s = 0.0;
        double fraction = 0.0; // overall, 0..1, weighted by points
        double eta_s = -1.0; // < 0 while unknown
        bool stop_after_current = false;
        bool stop_now = false;
    };

    //! Share of one recording's processing that is done, 0..1. Odometry is ~90 % of the time, the export the rest
    //! (§9 of DESIGN.md: ~21 min odometry, ~2.5 min export).
    double pipelineFraction(const PipelineStatus& status);

    //! Processes one recording. The batch runs a Pipeline; tests use a fake.
    class RecordingJob
    {
    public:
        virtual ~RecordingJob() = default;
        virtual bool start(const PipelineRequest& request, std::string& error) = 0;
        virtual bool busy() const = 0;
        virtual PipelineStatus status() const = 0;
        virtual void cancel() = 0;
        virtual std::vector<std::string> logTail() const
        {
            return {};
        }
    };

    using RecordingJobFactory = std::function<std::unique_ptr<RecordingJob>()>;

    //! Runs a batch on a worker thread, one recording after another. A failed recording is marked failed and the
    //! batch goes on. Each finished recording gets a done marker; recordings with a matching marker are skipped, so
    //! restarting the same batch continues where it stopped. batch_summary.csv is rewritten after every recording,
    //! so it is current even if the batch never ends normally. System sleep is prevented while it runs.
    class BatchRunner
    {
    public:
        //! An empty factory runs the real Pipeline.
        explicit BatchRunner(RecordingJobFactory factory = {});
        ~BatchRunner(); // stops now and joins

        bool start(const BatchRequest& request, std::string& error);
        //! Lets the running recording finish, then stops.
        void stopAfterCurrent();
        //! Cancels the running recording (its unfinished results are removed, as in single mode) and stops.
        void stopNow();
        bool busy() const
        {
            return busy_.load();
        }
        //! Blocks until the worker thread has finished.
        void wait();

        BatchStatus status() const;
        const BatchRequest& request() const
        {
            return request_; // only changed by start(), which refuses while busy
        }
        //! Last lines of batch.log, oldest first.
        std::vector<std::string> logTail() const;
        //! Last lines of the running recording's own log.
        std::vector<std::string> currentLogTail() const;

        static constexpr const char* kLogFileName = "batch.log";
        static constexpr const char* kSummaryFileName = "batch_summary.csv";

    private:
        void run();
        void processItem(size_t index);
        void log(const std::string& line);
        void writeSummary();
        void updateProgress(); // called with mutex_ held

        RecordingJobFactory factory_;
        BatchRequest request_;
        std::thread thread_;
        std::atomic<bool> busy_{ false };

        mutable std::mutex mutex_;
        BatchStatus status_;
        std::vector<double> weights_; // per item, for the overall fraction
        double processing_s_ = 0.0; // time spent in finished recordings that really ran (for the ETA)
        double processed_weight_ = 0.0; // their weight
        std::chrono::steady_clock::time_point started_;
        std::chrono::steady_clock::time_point item_started_;
        RecordingJob* job_ = nullptr; // valid while a recording runs; guarded by mutex_
        std::deque<std::string> log_tail_;
        std::ofstream log_file_; // only touched by the worker thread
    };
} // namespace easy_gui
