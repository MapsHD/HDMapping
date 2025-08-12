# Windows Build System Enhancement and Cross-Platform Optimization (with ARM Support)

Fixes #162 — Performance regression and Windows build system improvements, plus a comprehensive CPU optimization system for x86-64 (AMD/Intel) and ARM.

## Summary
This PR resolves the performance regression reported in #162 by replacing problematic, globally-applied optimization flags with a configurable, architecture-aware CPU optimization system. It enhances Windows build workflows, adds multi-language documentation, and extends support to ARM architectures while preserving cross-platform compatibility.

## Key Features

- Configurable CPU optimization modes: AUTO, AMD, INTEL, ARM, GENERIC
- Automatic CPU/architecture detection with safe fallbacks
- Cross-platform support (Windows/MSVC and Linux with GCC/Clang)
- Windows build automation via PowerShell and improved .gitignore
- ARM64/ARM32 support (NEON/Advanced SIMD) with MSVC and GCC/Clang

## Problem Analysis (Root Cause of #162)

Earlier changes introduced aggressive global flags (e.g., MSVC `/GL` + `/LTCG`, forced AVX/AVX2; GCC/Clang `-O3 -march=x86-64`) applied to all targets, including dependencies. This caused:
- Build instability and longer link times
- Potential runtime issues on CPUs lacking advanced instruction sets
- Architecture lock-in and conflicts with future optimization systems

## Solution Overview

1. Replace hard-coded flags with a configurable system (HD_CPU_OPTIMIZATION)
2. Provide safe defaults (AUTO), and explicit AMD/INTEL/ARM/GENERIC modes
3. Add ARM support (ARM64/AArch64, ARMv7) with NEON where available
4. Keep changes scoped to our targets; avoid global flags affecting 3rd party code
5. Improve Windows automation and docs (EN/RO/PL) for accessibility

## Usage

### Quick start
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

### Advanced configuration
```bash
# Force specific optimizations
cmake .. -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=AUTO     # default
cmake .. -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=GENERIC  # original behavior
cmake .. -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=AMD      # AMD
cmake .. -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=INTEL    # Intel
cmake .. -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=ARM      # ARM/ARM64
```

## Technical Details

### Detection and flags (illustrative)
```cmake
# x86-64 AUTO picks safe /Oi /Ot /Oy (MSVC) or -O2/-mtune=native (GCC/Clang)
# ARM64: -march=armv8-a+simd (GCC/Clang) or default NEON on MSVC; ARMv7: -mfpu=neon
```

### Build automation (Windows)
- `build_all_vs17.ps1` builds GENERIC/AMD/INTEL (and can be extended with ARM)
- Standardized binary naming for clarity

## Backward Compatibility

- GENERIC mode preserves the original v0.84.0 behavior
- No breaking changes to existing workflows

## Testing Performed

- Builds validated for GENERIC, AMD, INTEL, AUTO on Windows; ARM validated where available
- Cross-platform CMake configuration validated on Linux
- PowerShell automation tested; documentation validated

## Files Changed (high level)

- `CMakeLists.txt`: Add HD_CPU_OPTIMIZATION, refine flags; version set to v0.85.0
- `.gitignore`: Broader coverage for build outputs
- `build_all_vs17.ps1`: Multi-variant builds; standardized outputs
- Docs: EN/RO/PL Windows guides; CPU optimization guidance

## Benefits

- Better performance via CPU-aware builds
- Safer, maintainable flags; fewer surprises in 3rd parties
- Broader platform reach with ARM support
- Simpler Windows onboarding and repeatable builds

---

This PR strengthens HDMapping’s build system, fixes #162, and adds a flexible optimization framework that scales across AMD, Intel, and ARM while keeping compatibility intact.
