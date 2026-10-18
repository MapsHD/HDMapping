#include "child_process.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

namespace fs = std::filesystem;

namespace easy_gui
{
#ifdef _WIN32
    struct ChildProcess::Impl
    {
        HANDLE process = nullptr;
        HANDLE job = nullptr;
        HANDLE read_pipe = nullptr;
        bool waited = false;
        int exit_code = -1;
    };

    namespace
    {
        std::wstring widen(const std::string& s)
        {
            if (s.empty())
                return {};
            const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
            std::wstring out(n, L'\0');
            MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), out.data(), n);
            return out;
        }

        // Quoting rules of CommandLineToArgvW / the MSVC runtime: backslashes are literal except directly before a
        // double quote, where they have to be doubled.
        std::wstring quoteArgument(const std::wstring& arg)
        {
            if (!arg.empty() && arg.find_first_of(L" \t\n\v\"") == std::wstring::npos)
                return arg;
            std::wstring out = L"\"";
            for (auto it = arg.begin();; ++it)
            {
                size_t backslashes = 0;
                while (it != arg.end() && *it == L'\\')
                {
                    ++it;
                    ++backslashes;
                }
                if (it == arg.end())
                {
                    out.append(backslashes * 2, L'\\');
                    break;
                }
                if (*it == L'"')
                    out.append(backslashes * 2 + 1, L'\\');
                else
                    out.append(backslashes, L'\\');
                out.push_back(*it);
            }
            out.push_back(L'"');
            return out;
        }

        std::string lastErrorMessage()
        {
            return "Windows error " + std::to_string(GetLastError());
        }
    } // namespace

    ChildProcess::ChildProcess()
        : impl_(new Impl)
    {
    }

    ChildProcess::~ChildProcess()
    {
        if (impl_->process && !impl_->waited)
        {
            kill();
            wait();
        }
        if (impl_->read_pipe)
            CloseHandle(impl_->read_pipe);
        if (impl_->process)
            CloseHandle(impl_->process);
        if (impl_->job)
            CloseHandle(impl_->job);
        delete impl_;
    }

    bool ChildProcess::start(const fs::path& executable, const std::vector<std::string>& arguments, std::string& error)
    {
        SECURITY_ATTRIBUTES sa{ sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE };
        HANDLE read_pipe = nullptr;
        HANDLE write_pipe = nullptr;
        if (!CreatePipe(&read_pipe, &write_pipe, &sa, 0))
        {
            error = "Could not create a pipe (" + lastErrorMessage() + ")";
            return false;
        }
        SetHandleInformation(read_pipe, HANDLE_FLAG_INHERIT, 0);

        HANDLE nul = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, OPEN_EXISTING, 0, nullptr);

        // The job object makes kill() take down the whole process tree and also kills the child if Easy GUI itself
        // dies (JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE).
        HANDLE job = CreateJobObjectW(nullptr, nullptr);
        if (job)
        {
            JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
            info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
            SetInformationJobObject(job, JobObjectExtendedLimitInformation, &info, sizeof(info));
        }

        std::wstring command_line = quoteArgument(executable.wstring());
        for (const auto& arg : arguments)
            command_line += L" " + quoteArgument(widen(arg));

        STARTUPINFOW si{};
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdInput = nul;
        si.hStdOutput = write_pipe;
        si.hStdError = write_pipe;

        PROCESS_INFORMATION pi{};
        const BOOL ok = CreateProcessW(
            executable.wstring().c_str(),
            command_line.data(),
            nullptr,
            nullptr,
            TRUE,
            CREATE_NO_WINDOW | CREATE_SUSPENDED,
            nullptr,
            nullptr,
            &si,
            &pi);
        const std::string create_error = ok ? std::string() : lastErrorMessage();

        CloseHandle(write_pipe);
        if (nul != INVALID_HANDLE_VALUE)
            CloseHandle(nul);

        if (!ok)
        {
            CloseHandle(read_pipe);
            if (job)
                CloseHandle(job);
            error = "Could not start '" + executable.string() + "' (" + create_error + ")";
            return false;
        }

        if (job)
            AssignProcessToJobObject(job, pi.hProcess);
        ResumeThread(pi.hThread);
        CloseHandle(pi.hThread);

        impl_->process = pi.hProcess;
        impl_->job = job;
        impl_->read_pipe = read_pipe;
        return true;
    }

    bool ChildProcess::readLine(std::string& line)
    {
        for (;;)
        {
            const auto newline = buffer_.find('\n');
            if (newline != std::string::npos)
            {
                line = buffer_.substr(0, newline);
                buffer_.erase(0, newline + 1);
                std::erase(line, '\r');
                return true;
            }
            if (eof_ || !impl_->read_pipe)
            {
                if (buffer_.empty())
                    return false;
                line.swap(buffer_);
                buffer_.clear();
                std::erase(line, '\r');
                return true;
            }
            char chunk[4096];
            DWORD n = 0;
            if (!ReadFile(impl_->read_pipe, chunk, sizeof(chunk), &n, nullptr) || n == 0)
                eof_ = true;
            else
                buffer_.append(chunk, n);
        }
    }

    void ChildProcess::kill()
    {
        killed_ = true;
        if (impl_->job)
            TerminateJobObject(impl_->job, 1);
        else if (impl_->process)
            TerminateProcess(impl_->process, 1);
    }

    int ChildProcess::wait()
    {
        if (!impl_->process)
            return -1;
        if (!impl_->waited)
        {
            WaitForSingleObject(impl_->process, INFINITE);
            DWORD code = 0;
            impl_->exit_code = GetExitCodeProcess(impl_->process, &code) ? static_cast<int>(code) : -1;
            impl_->waited = true;
        }
        return killed_ ? -1 : impl_->exit_code;
    }

    fs::path currentExecutableDirectory()
    {
        std::wstring buffer(32768, L'\0');
        const DWORD n = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
        if (n == 0 || n >= buffer.size())
            return {};
        buffer.resize(n);
        return fs::path(buffer).parent_path();
    }

    void attachToParentConsole()
    {
        if (!AttachConsole(ATTACH_PARENT_PROCESS))
            return;
        FILE* stream = nullptr;
        freopen_s(&stream, "CONOUT$", "w", stdout);
        freopen_s(&stream, "CONOUT$", "w", stderr);
    }

#else // POSIX

    struct ChildProcess::Impl
    {
        pid_t pid = -1;
        int read_fd = -1;
        std::atomic<bool> reaped{ false };
        int exit_code = -1;
    };

    ChildProcess::ChildProcess()
        : impl_(new Impl)
    {
    }

    ChildProcess::~ChildProcess()
    {
        if (impl_->pid > 0 && !impl_->reaped)
        {
            kill();
            wait();
        }
        if (impl_->read_fd >= 0)
            close(impl_->read_fd);
        delete impl_;
    }

    bool ChildProcess::start(const fs::path& executable, const std::vector<std::string>& arguments, std::string& error)
    {
        int fds[2];
        if (pipe(fds) != 0)
        {
            error = std::string("Could not create a pipe (") + std::strerror(errno) + ")";
            return false;
        }
        // Only the child's copy of the write end may stay open, or the parent never sees end-of-output.
        fcntl(fds[0], F_SETFD, FD_CLOEXEC);
        fcntl(fds[1], F_SETFD, FD_CLOEXEC);

        posix_spawn_file_actions_t actions;
        posix_spawn_file_actions_init(&actions);
        posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
        posix_spawn_file_actions_adddup2(&actions, fds[1], STDOUT_FILENO);
        posix_spawn_file_actions_adddup2(&actions, fds[1], STDERR_FILENO);

        // Own process group, so kill() can signal everything the child starts.
        posix_spawnattr_t attributes;
        posix_spawnattr_init(&attributes);
        posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
        posix_spawnattr_setpgroup(&attributes, 0);

        const std::string exe = executable.string();
        std::vector<std::string> argv_storage;
        argv_storage.reserve(arguments.size() + 1);
        argv_storage.push_back(exe);
        argv_storage.insert(argv_storage.end(), arguments.begin(), arguments.end());
        std::vector<char*> argv;
        for (auto& a : argv_storage)
            argv.push_back(a.data());
        argv.push_back(nullptr);

        pid_t pid = -1;
        const int rc = posix_spawn(&pid, exe.c_str(), &actions, &attributes, argv.data(), environ);

        posix_spawn_file_actions_destroy(&actions);
        posix_spawnattr_destroy(&attributes);
        close(fds[1]);

        if (rc != 0)
        {
            close(fds[0]);
            error = "Could not start '" + exe + "' (" + std::strerror(rc) + ")";
            return false;
        }

        impl_->pid = pid;
        impl_->read_fd = fds[0];
        return true;
    }

    bool ChildProcess::readLine(std::string& line)
    {
        for (;;)
        {
            const auto newline = buffer_.find('\n');
            if (newline != std::string::npos)
            {
                line = buffer_.substr(0, newline);
                buffer_.erase(0, newline + 1);
                std::erase(line, '\r');
                return true;
            }
            if (eof_ || impl_->read_fd < 0)
            {
                if (buffer_.empty())
                    return false;
                line.swap(buffer_);
                buffer_.clear();
                std::erase(line, '\r');
                return true;
            }
            char chunk[4096];
            const ssize_t n = read(impl_->read_fd, chunk, sizeof(chunk));
            if (n > 0)
                buffer_.append(chunk, static_cast<size_t>(n));
            else if (n < 0 && errno == EINTR)
                continue;
            else
                eof_ = true;
        }
    }

    void ChildProcess::kill()
    {
        killed_ = true;
        // SIGKILL rather than SIGTERM: step 1 installs no handlers, so there is nothing to shut down gracefully, and
        // a stuck TBB worker must not be able to delay the cancel.
        if (impl_->pid > 0 && !impl_->reaped)
            ::kill(-impl_->pid, SIGKILL);
    }

    int ChildProcess::wait()
    {
        if (impl_->pid <= 0)
            return -1;
        if (!impl_->reaped)
        {
            int status = 0;
            pid_t r;
            do
                r = waitpid(impl_->pid, &status, 0);
            while (r < 0 && errno == EINTR);
            impl_->exit_code = (r == impl_->pid && WIFEXITED(status)) ? WEXITSTATUS(status) : -1;
            impl_->reaped = true;
        }
        return impl_->exit_code;
    }

    fs::path currentExecutableDirectory()
    {
#ifdef __APPLE__
        uint32_t size = 0;
        _NSGetExecutablePath(nullptr, &size);
        std::string buffer(size, '\0');
        if (_NSGetExecutablePath(buffer.data(), &size) != 0)
            return {};
        buffer.resize(std::strlen(buffer.c_str()));
        std::error_code ec;
        const fs::path resolved = fs::weakly_canonical(buffer, ec);
        return (ec ? fs::path(buffer) : resolved).parent_path();
#else
        std::error_code ec;
        const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
        return ec ? fs::path() : exe.parent_path();
#endif
    }

    void attachToParentConsole()
    {
    }
#endif
} // namespace easy_gui
