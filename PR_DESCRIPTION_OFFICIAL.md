# Fix Performance Regression: Remove Problematic AMD64 Optimizations and Integrate All PRs

## Fixes #162

This PR addresses the performance regression reported in issue #162 ("Performance Regression in v0.85.0").

## Problem Description

This PR addresses a critical performance regression introduced by commit `3079bd1` ("Add AMD64 optimizations for better performance") in PR #150, which paradoxically **decreased** performance instead of improving it, as documented in issue #162.

## Root Cause Analysis

Commit `3079bd1` introduced several problematic optimization flags:

### Issues Identified:

1. **Aggressive Global Flags**: Applied optimization flags globally to ALL projects, including dependencies and 3rd party libraries
   - MSVC: `/O2 /Oi /Ot /Oy /GL` + `/LTCG`  
   - GCC/Clang: `-O3 -march=x86-64 -mtune=generic`

2. **Forced AVX/AVX2 Instructions**: Automatically enabled advanced CPU instructions without runtime detection
   - Risk of crashes on older CPUs that don't support these instructions
   - No fallback mechanism for incompatible hardware

3. **Link-Time Code Generation (LTCG) Issues**: 
   - Can cause build failures with certain library combinations
   - Increases compilation time significantly
   - May interfere with debugging and profiling

4. **Architecture Lock-in**: Hard-coded x86-64 optimizations
   - Breaks compatibility with other architectures (ARM, etc.)
   - Conflicts with the newer configurable CPU optimization system from PR #154

## Performance Impact

Through systematic testing, we confirmed that removing commit `3079bd1` resulted in:
- ✅ **Better runtime performance** 
- ✅ **Stable builds across all CPU architectures**
- ✅ **Compatibility with the new optimization system from PR #154**

## Solution Implemented

1. **Selective Removal**: Removed only the problematic commit `3079bd1` from PR #150 while preserving all other valuable changes

2. **Complete Integration**: Successfully applied ALL PRs from #150 through #160:
   - PR #150: SOTA algorithm compatibility (minus problematic optimization)
   - PR #151: Freeglut submodule fixes  
   - PR #152: Submodule version management
   - PR #154: **Proper configurable CPU optimization system** 
   - PR #156: TOML issue fixes
   - PR #158: ARM-only optimizations
   - PR #147: Missing include fixes
   - PR #160: Code cleaning

3. **Build Automation**: Created comprehensive build scripts (`build_all_vs17.ps1`) supporting all CPU architectures

4. **Conflict Resolution**: Manually resolved all merge conflicts, especially in CMakeLists.txt

## Why This Approach Works

The newer **PR #154** provides a much better optimization solution:
- ✅ **Configurable per architecture** (Generic, Intel, AMD, ARM)
- ✅ **Runtime CPU detection**
- ✅ **Proper fallback mechanisms**  
- ✅ **No forced global flags**
- ✅ **Build-time architecture selection**

## Testing Performed

- ✅ Successful builds on all CPU architectures (Generic, Intel, AMD, ARM)
- ✅ Performance testing with and without the problematic commit
- ✅ Verification that all PR features work correctly
- ✅ Build automation scripts tested and functional

## Lessons Learned

1. **Avoid global optimization flags** - they affect all dependencies unpredictably
2. **Test performance impact** - "optimization" commits should be benchmarked  
3. **Architecture-specific code** should be configurable, not hardcoded
4. **AVX/AVX2 instructions** require runtime detection for compatibility

## Files Changed

- Removed problematic optimizations from `CMakeLists.txt`
- **Updated version to v0.85.0** to match git tag (was incorrectly set to v0.84.0)
- Added comprehensive build automation
- Integrated all features from PRs #150-#160
- Added performance testing and documentation
- **Added comprehensive Windows build guides** (Romanian, English, Polish)
- **Added PowerShell build automation scripts**

## Closes #162

This PR directly resolves the performance regression reported in issue #162 by removing the problematic optimization flags that caused the performance degradation in v0.85.0.

## Ready for Integration

This branch (`v0_84_to_head_bisect`) is now:
- ✅ **Performance-optimized** (without the regression)
- ✅ **Fully up-to-date** with all latest PRs  
- ✅ **Build-tested** across all architectures
- ✅ **Conflict-free** and ready for merge

The codebase is now in a significantly better state than before, with proper optimization systems and all latest features integrated.
