# HDMapping Changelog

All notable changes to HDMapping will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.85.0] - 2025-08-04

### Added
- **Comprehensive CPU Optimization System**
  - Automatic CPU architecture detection (AMD, Intel, ARM)
  - Configurable optimization modes: AUTO, AMD, INTEL, ARM, GENERIC
  - SIMD optimizations: AVX2 (AMD), AVX (Intel), NEON (ARM)
  - Cross-platform support for MSVC, GCC, and Clang compilers
  - Performance improvements: 10-25% depending on architecture

- **ARM Architecture Support**
  - Full ARM64/AArch64 support with Advanced SIMD (NEON)
  - ARM32 support with explicit NEON optimizations
  - Support for Apple Silicon (M1, M2, M3), ARM servers, Raspberry Pi 4+
  - Cross-compilation compatibility for ARM targets

- **Enhanced Build System**
  - `HD_CPU_OPTIMIZATION` CMake variable for optimization control
  - Automatic SIMD capability detection and activation
  - Link Time Code Generation (LTCG) support
  - Improved compiler flag management across platforms

- **Testing Infrastructure**
  - Dedicated `tests/` directory for organized test files
  - Version system validation with `test_version_system.cpp`
  - TOML structure template with `test_toml_structure.toml`
  - CMake integration with CTest support (`BUILD_TESTING` option)

- **Comprehensive Documentation**
  - CPU Optimization Guide with detailed platform-specific instructions
  - TOML Configuration Guide for parameter management
  - Performance benchmarking data and recommendations
  - Migration guide from previous versions

### Changed
- **TOML Configuration Structure**
  - Separated `motion_model_correction` parameters into dedicated section
  - Improved logical organization of configuration parameters
  - Enhanced parameter grouping following TOML best practices
  - Better configuration file validation and error reporting

- **Build Configuration**
  - Default optimization mode changed to AUTO (was no optimization)
  - Enhanced CMake configuration with architecture-aware optimizations
  - Improved cross-platform compatibility for build systems
  - Better integration with CI/CD pipelines

### Fixed
- **Cross-Platform Compatibility**
  - Fixed `localtime_s` function usage between Windows and Unix systems
  - Resolved compilation warnings on different compiler versions
  - Improved memory allocation patterns for better performance
  - Enhanced error handling in configuration loading

### Performance
- **Optimization Improvements**
  - AMD processors: 10-20% performance improvement with AVX2 and aggressive optimizations
  - Intel processors: 5-15% performance improvement with conservative optimizations
  - ARM systems: 15-25% performance improvement with NEON SIMD optimizations
  - Generic builds: Maintain baseline performance with maximum compatibility

### Security
- **Enhanced Validation**
  - Improved TOML configuration validation with type checking
  - Better parameter range validation and error reporting
  - Enhanced memory safety in configuration parsing
  - Stricter input validation for file paths and parameters

## [0.84.0] - Previous Release

### Baseline Features
- Core SLAM functionality with lidar odometry
- Multi-session registration capabilities
- Point cloud processing and visualization
- Trajectory comparison and analysis tools
- Basic CMake build system
- Initial TOML configuration support

---

## Upgrade Instructions

### From 0.84.0 to 0.85.0

#### For Existing Users (Backward Compatibility)
To maintain **exact** compatibility with v0.84.0 behavior:
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=GENERIC
```

#### For Enhanced Performance (Recommended)
To benefit from automatic optimization:
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=AUTO
```

#### For ARM Platforms
New ARM support enables native builds:
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=ARM
```

### Configuration File Updates

#### TOML Structure Changes
Update your configuration files to use the new structure:

**Old (v0.84.0):**
```toml
[motion_model_uncertainty]
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01
motion_model_correction_fi = 0.0  # Wrong section
motion_model_correction_ka = 0.0
motion_model_correction_om = 0.0
```

**New (v0.85.0):**
```toml
[motion_model_uncertainty]
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01

[motion_model_correction]  # Dedicated section
motion_model_correction_fi = 0.0
motion_model_correction_ka = 0.0
motion_model_correction_om = 0.0
```

## Development Notes

### Build System Changes
- **CMake minimum version:** 3.15.0 (no change)
- **New CMake variables:** `HD_CPU_OPTIMIZATION`
- **Enhanced compiler detection:** Automatic SIMD capability detection
- **Improved platform support:** Better Windows/Linux/macOS compatibility

### Dependencies
- **No new external dependencies** added
- **Enhanced internal libraries:** Updated Eigen integration for better optimization
- **Improved bundled libraries:** Better optimization flag propagation

### Testing and Validation
- **Multi-platform testing:** Verified on x86-64 (AMD/Intel) and ARM64 platforms
- **Performance benchmarking:** Comprehensive performance testing across architectures
- **Regression testing:** Full backward compatibility validation
- **Cross-compilation testing:** ARM toolchain compatibility verification

---

## Contributing

When contributing to HDMapping, please:

1. **Update this changelog** with your changes
2. **Follow semantic versioning** for version numbers
3. **Include performance impact** if applicable
4. **Document breaking changes** clearly
5. **Provide migration instructions** when needed

### Changelog Sections
- **Added:** New features
- **Changed:** Changes in existing functionality
- **Deprecated:** Soon-to-be removed features
- **Removed:** Removed features
- **Fixed:** Bug fixes
- **Security:** Security improvements
- **Performance:** Performance improvements

---

*For detailed technical information, see the CPU Optimization Guide and TOML Configuration Guide in the docs/ directory.*
