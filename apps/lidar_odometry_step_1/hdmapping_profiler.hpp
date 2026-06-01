#pragma once

// HDMapping profiler backend — chosen at CMake configure time via HDMAPPING_PROFILER=NONE|UTL|TRACY
//
// Usage in source files:
//   HDMAP_ZONE_SCOPE("label")          — scoped zone, ends when enclosing scope exits
//   HDMAP_ZONE_BEGIN(id, "label")      — manual zone begin  (id must be unique in scope)
//   HDMAP_ZONE_END(id)                 — manual zone end

#if defined(HDMAPPING_PROFILER_TRACY)

#pragma message("HDMapping profiler: Tracy enabled")
#include <tracy/Tracy.hpp>
#include <tracy/TracyC.h>
// ZoneNamedN lets us embed __LINE__ in the variable name, avoiding clashes
// when multiple HDMAP_ZONE_SCOPE appear in the same function scope.
#define HDMAP_ZONE_SCOPE(name) ZoneNamedN(TracyConcat(hdmap_zone_, __LINE__), name, true)
#define HDMAP_ZONE_BEGIN(id, name) TracyCZoneN(hdmap_ctx_##id, name, true)
#define HDMAP_ZONE_END(id) TracyCZoneEnd(hdmap_ctx_##id)
#define HDMAP_FRAME_MARK FrameMark
#define HDMAP_MESSAGE(msg) TracyMessageL(msg)
#define HDMAP_MESSAGE_STR(str) TracyMessage((str).c_str(), (str).size())
#define HDMAP_PLOT(name, val) TracyPlot(name, static_cast<int64_t>(val))

#elif defined(HDMAPPING_PROFILER_UTL)

#include <UTL/profiler.hpp>
#define HDMAP_ZONE_SCOPE(name) UTL_PROFILER_SCOPE(name)
#define HDMAP_ZONE_BEGIN(id, name) UTL_PROFILER_BEGIN(id, name)
#define HDMAP_ZONE_END(id) UTL_PROFILER_END(id)
#define HDMAP_FRAME_MARK
#define HDMAP_MESSAGE(msg)
#define HDMAP_MESSAGE_STR(str)
#define HDMAP_PLOT(name, val)

#else

#define HDMAP_ZONE_SCOPE(name)
#define HDMAP_ZONE_BEGIN(id, name)
#define HDMAP_ZONE_END(id)
#define HDMAP_FRAME_MARK
#define HDMAP_MESSAGE(msg)
#define HDMAP_MESSAGE_STR(str)
#define HDMAP_PLOT(name, val)

#endif