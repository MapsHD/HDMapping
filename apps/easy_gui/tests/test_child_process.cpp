#include <doctest.h>

#include "child_process.h"

#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>

using easy_gui::ChildProcess;

namespace
{
    std::vector<std::string> readAll(ChildProcess& child)
    {
        std::vector<std::string> lines;
        std::string line;
        while (child.readLine(line))
            lines.push_back(line);
        return lines;
    }
} // namespace

TEST_CASE("arguments reach the child unchanged (spaces, quotes, backslashes, empty)")
{
    // easy_gui_tests itself is the child (--echo-args, see test_main.cpp). On Windows this checks the command line
    // quoting, which output folders like "Documents\Easy Forest HDMapping\..." depend on.
#ifdef _WIN32
    const auto self = easy_gui::currentExecutableDirectory() / "easy_gui_tests.exe";
#else
    const auto self = easy_gui::currentExecutableDirectory() / "easy_gui_tests";
#endif
    const std::vector<std::string> args = { "--echo-args",          "plain",      "with space", "", "quote\"inside",
                                            "C:\\dir with space\\", "trailing\\", "a\\\\\"b" };
    ChildProcess child;
    std::string error;
    REQUIRE_MESSAGE(child.start(self, args, error), error);
    const auto lines = readAll(child);
    CHECK(child.wait() == 0);
    REQUIRE(lines.size() == args.size() - 1);
    for (size_t i = 1; i < args.size(); i++)
        CHECK(lines[i - 1] == "[" + args[i] + "]");
}

#ifndef _WIN32 // uses /bin/sh

TEST_CASE("stdout and stderr are merged line by line and the exit code is returned")
{
    ChildProcess child;
    std::string error;
    REQUIRE(child.start("/bin/sh", { "-c", "echo one; echo two 1>&2; printf 'three\\r\\nfour'; exit 3" }, error));
    std::vector<std::string> lines;
    std::string line;
    while (child.readLine(line))
        lines.push_back(line);
    CHECK(child.wait() == 3);
    REQUIRE(lines.size() == 4);
    CHECK(lines[0] == "one");
    CHECK(lines[1] == "two");
    CHECK(lines[2] == "three"); // '\r' removed
    CHECK(lines[3] == "four"); // last line without a newline
}

TEST_CASE("stdin is closed, so a program waiting for input does not hang")
{
    // load_data does std::cin.get() before std::exit on a calibration mismatch.
    ChildProcess child;
    std::string error;
    REQUIRE(child.start("/bin/sh", { "-c", "read x; echo got-eof" }, error));
    std::string line;
    REQUIRE(child.readLine(line));
    CHECK(line == "got-eof");
    CHECK(child.wait() != -1);
}

TEST_CASE("kill stops the child and its process group promptly")
{
    ChildProcess child;
    std::string error;
    // The background sleep is a grandchild: it must die too, or it would hold the pipe open.
    REQUIRE(child.start("/bin/sh", { "-c", "sleep 30 & echo started; sleep 30" }, error));
    std::string line;
    REQUIRE(child.readLine(line));
    CHECK(line == "started");

    const auto t0 = std::chrono::steady_clock::now();
    std::thread killer(
        [&]
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            child.kill();
        });
    while (child.readLine(line))
    {
    }
    killer.join();
    CHECK(child.wait() == -1);
    CHECK(child.wasKilled());
    CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5));
}

TEST_CASE("starting a missing program fails with a message")
{
    ChildProcess child;
    std::string error;
    CHECK_FALSE(child.start("/nonexistent/program", {}, error));
    CHECK_FALSE(error.empty());
}

#else // cmd.exe

namespace
{
    std::filesystem::path cmdExe()
    {
        const char* comspec = std::getenv("ComSpec");
        return (comspec && *comspec) ? std::filesystem::path(comspec) : std::filesystem::path("C:\\Windows\\System32\\cmd.exe");
    }
} // namespace

TEST_CASE("stdout and stderr are merged line by line and the exit code is returned")
{
    ChildProcess child;
    std::string error;
    // Redirection first: in "echo two 1>&2" cmd would print the space before the redirection too.
    REQUIRE(child.start(cmdExe(), { "/c", "echo one& 1>&2 echo two& exit /b 3" }, error));
    const auto lines = readAll(child);
    CHECK(child.wait() == 3);
    REQUIRE(lines.size() == 2);
    CHECK(lines[0] == "one"); // '\r' of the CRLF removed
    CHECK(lines[1] == "two");
}

TEST_CASE("stdin is closed, so a program waiting for input does not hang")
{
    ChildProcess child;
    std::string error;
    REQUIRE(child.start(cmdExe(), { "/c", "set /p x=& echo got-eof" }, error));
    const auto lines = readAll(child);
    REQUIRE(!lines.empty());
    CHECK(lines.back() == "got-eof");
    CHECK(child.wait() != -1);
}

TEST_CASE("kill stops the child and everything it started promptly")
{
    ChildProcess child;
    std::string error;
    // ping is a grandchild holding the pipe's write end (stderr): only the job object takes it down too.
    REQUIRE(child.start(cmdExe(), { "/c", "echo started& ping -n 30 127.0.0.1 >nul" }, error));
    std::string line;
    REQUIRE(child.readLine(line));
    CHECK(line == "started");

    const auto t0 = std::chrono::steady_clock::now();
    std::thread killer(
        [&]
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            child.kill();
        });
    while (child.readLine(line))
    {
    }
    killer.join();
    CHECK(child.wait() == -1);
    CHECK(child.wasKilled());
    CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5));
}

TEST_CASE("starting a missing program fails with a message")
{
    ChildProcess child;
    std::string error;
    CHECK_FALSE(child.start("C:\\nonexistent\\program.exe", {}, error));
    CHECK_FALSE(error.empty());
}

#endif
