#pragma once

#include <cstdlib>
#include <iostream>

// Unlike assert(), HDMAPPING_ASSERT is never compiled out by NDEBUG: a failed invariant is
// always reported. Debug builds abort immediately so the bug is impossible to miss during
// development; release builds (NDEBUG) only log a warning and continue, since aborting a
// shipped app on an invariant violation is worse for users than limping on in a degraded
// state (e.g. a swallowed std::out_of_range from an unchecked session/point index).
//
// Define HDMAPPING_ASSERT_ALWAYS_ABORT (e.g. via target_compile_definitions on one target)
// to force the abort even with NDEBUG defined -- for debugging a RelWithDebInfo/Release
// build without flipping NDEBUG, and everything else gated on it, for the whole project.
#if !defined(NDEBUG) || defined(HDMAPPING_ASSERT_ALWAYS_ABORT)
#define HDMAPPING_ASSERT_ABORT() std::abort()
#else
#define HDMAPPING_ASSERT_ABORT() ((void)0)
#endif

// message is optional: HDMAPPING_ASSERT(cond) or HDMAPPING_ASSERT(cond, "why")
#define HDMAPPING_ASSERT(condition, ...)                                                                                                   \
    do                                                                                                                                     \
    {                                                                                                                                      \
        if (!(condition))                                                                                                                  \
        {                                                                                                                                  \
            std::cerr << "HDMAPPING_ASSERT failed: " #condition << " at " << __FILE__ << ":"                                               \
                      << __LINE__ __VA_OPT__(<< " -- " << (__VA_ARGS__)) << std::endl;                                                     \
            HDMAPPING_ASSERT_ABORT();                                                                                                      \
        }                                                                                                                                  \
    } while (false)
