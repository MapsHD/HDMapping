#include <doctest.h>

#include "child_process.h"

#include <chrono>
#include <thread>
#include <vector>

using easy_gui::ChildProcess;

#ifndef _WIN32 // uses /bin/sh; the Windows implementation is exercised by running Easy GUI

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

#endif
