#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace easy_gui
{
    //! Runs one external program with stdout and stderr merged into a single pipe and stdin closed.
    //!
    //! Easy GUI runs lidar odometry as a child process instead of a thread because the step 1 sources have
    //! abort()/exit() paths (corrupt LAZ, calibration mismatch, out of memory) that would take the whole GUI down,
    //! and because compute_step_2 can only be paused, not cancelled. Killing the child is the cancel.
    //!
    //! stdin is closed on purpose: load_data waits on std::cin.get() before std::exit on a calibration mismatch, and
    //! an EOF stdin lets it exit immediately instead of hanging forever.
    //!
    //! readLine() is meant for one reader thread; kill() may be called from any other thread and makes a blocked
    //! readLine() return false once the child's end of the pipe closes.
    class ChildProcess
    {
    public:
        ChildProcess();
        ~ChildProcess(); // kills a still-running child and reaps it
        ChildProcess(const ChildProcess&) = delete;
        ChildProcess& operator=(const ChildProcess&) = delete;

        bool start(const std::filesystem::path& executable, const std::vector<std::string>& arguments, std::string& error);

        //! Next output line without its line terminator ('\r' characters are removed too: step 1 prints progress
        //! as "\rrunning iterations: ..."). Returns false at end of output.
        bool readLine(std::string& line);

        //! Terminates the child (on POSIX its whole process group, so helper processes go too). Safe to call twice.
        void kill();

        //! Waits for the child to exit and returns its exit code, or -1 if it was killed by a signal / could not be
        //! waited for.
        int wait();

        bool wasKilled() const
        {
            return killed_.load();
        }

    private:
        struct Impl;
        Impl* impl_ = nullptr;
        std::string buffer_;
        bool eof_ = false;
        std::atomic<bool> killed_{ false };
    };

    //! Directory of the running executable (empty if it cannot be determined).
    std::filesystem::path currentExecutableDirectory();
} // namespace easy_gui
