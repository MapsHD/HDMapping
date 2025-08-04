# CMake Configuration Guide for HDMapping

## Quick Reference

### CPU Optimization Options
```bash
# Option 1: Auto-detect (Recommended)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Option 2: Force AMD optimizations (Ryzen, EPYC)
cmake -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=AMD ..

# Option 3: Force Intel optimizations (Core, Xeon)
cmake -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=INTEL ..

# Option 4: Force ARM optimizations (Apple Silicon, ARM servers)
cmake -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=ARM ..

# Option 5: Generic compatibility (maximum compatibility)
cmake -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=GENERIC ..
```

#### What Each Optimization Does:
- **AUTO**: Detects CPU automatically (AMD→AVX2, Intel→AVX, ARM→NEON)
- **AMD**: Aggressive optimizations with `/Oi /Ot /Oy` (MSVC) or `-O3 -mavx2` (GCC)
- **INTEL**: Conservative optimizations with `/O2` (MSVC) or `-O2 -mavx` (GCC)
- **ARM**: NEON SIMD optimizations for ARM64/ARM32 architectures
- **GENERIC**: Safe `/O2` optimizations without SIMD assumptions

### Build Type Options
```bash
# Release build (optimized, no debug symbols)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Debug build (with debug symbols, no optimization)
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release with debug info (optimized + debug symbols)
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..

# Minimum size release (optimized for size)
cmake -DCMAKE_BUILD_TYPE=MinSizeRel ..
```

### Library Options
```bash
# Use bundled libraries (default, recommended)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Use system libraries (for Debian packages)
cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_WITH_BUNDLED_FREEGLUT=OFF \
    -DBUILD_WITH_BUNDLED_EIGEN=OFF \
    -DBUILD_WITH_BUNDLED_LIBLASZIP=OFF ..
```

### Advanced Performance Options
```bash
# Enable all optimizations + profiling support
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=AUTO \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG -g" ..

# Maximum performance build (no safety checks)
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=AMD \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG -ffast-math" ..

# Cross-compilation for ARM64
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=ARM \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ ..
```

### Python Bindings
```bash
# Enable Python bindings
cmake -DCMAKE_BUILD_TYPE=Release -DPYBIND=ON ..

# Python bindings with specific Python version
cmake -DCMAKE_BUILD_TYPE=Release \
    -DPYBIND=ON \
    -DPYTHON_EXECUTABLE=/usr/bin/python3.9 ..
```

### Complete Configuration Examples

#### Development Build (AMD CPU)
```bash
mkdir build-dev
cd build-dev
cmake -DCMAKE_BUILD_TYPE=Debug \
    -DHD_CPU_OPTIMIZATION=AMD \
    -DPYBIND=ON ..
cmake --build . --config Debug
```

#### Development Build (ARM CPU)  
```bash
mkdir build-arm-dev
cd build-arm-dev
cmake -DCMAKE_BUILD_TYPE=Debug \
    -DHD_CPU_OPTIMIZATION=ARM \
    -DPYBIND=ON ..
cmake --build . --config Debug
```

#### Production Build (Auto-optimized)
```bash
mkdir build-release
cd build-release
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=AUTO ..
cmake --build . --config Release
```

#### Distribution Package (Generic)
```bash
mkdir build-package
cd build-package
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=GENERIC \
    -DBUILD_WITH_BUNDLED_FREEGLUT=OFF \
    -DBUILD_WITH_BUNDLED_EIGEN=OFF \
    -DBUILD_WITH_BUNDLED_LIBLASZIP=OFF ..
cmake --build . --config Release
make pack
```

## Available CMake Variables

| Variable | Default | Options | Description |
|----------|---------|---------|-------------|
| `CMAKE_BUILD_TYPE` | Release | Debug, Release, RelWithDebInfo, MinSizeRel | Build configuration |
| `HD_CPU_OPTIMIZATION` | AUTO | AUTO, AMD, INTEL, ARM, GENERIC | CPU optimization strategy |
| `BUILD_WITH_BUNDLED_FREEGLUT` | ON | ON, OFF | Use bundled FreeGLUT library |
| `BUILD_WITH_BUNDLED_EIGEN` | ON | ON, OFF | Use bundled Eigen library |
| `BUILD_WITH_BUNDLED_LIBLASZIP` | ON | ON, OFF | Use bundled LASzip library |
| `PYBIND` | OFF | ON, OFF | Enable Python bindings |
| `BUILD_TESTING` | OFF | ON, OFF | Enable test suite compilation |

### Advanced Variables
| Variable | Default | Description |
|----------|---------|-------------|
| `CMAKE_CXX_COMPILER` | System default | Override C++ compiler (g++, clang++, etc.) |
| `CMAKE_C_COMPILER` | System default | Override C compiler (gcc, clang, etc.) |
| `CMAKE_INSTALL_PREFIX` | /usr/local | Installation directory prefix |
| `CMAKE_EXPORT_COMPILE_COMMANDS` | OFF | Generate compile_commands.json for IDEs |

### Performance Variables (Expert Use)
| Variable | Default | Description |
|----------|---------|-------------|
| `CMAKE_CXX_FLAGS_RELEASE` | Compiler default | Override release compilation flags |
| `CMAKE_EXE_LINKER_FLAGS` | System default | Additional linker flags |
| `CMAKE_INTERPROCEDURAL_OPTIMIZATION` | OFF | Enable LTO/IPO (Link Time Optimization) |

## Troubleshooting

### CMake Cache Issues
```bash
# Clear CMake cache and reconfigure
rm -rf CMakeCache.txt CMakeFiles/
cmake -DCMAKE_BUILD_TYPE=Release ..
```

### Check Current Configuration
```bash
# View all CMake variables
cmake -LA .

# View only HDMapping variables  
cmake -LA . | grep -E "(HD_|BUILD_WITH_|PYBIND)"
```

### Build Errors
1. **Optimization-related errors:** Try `HD_CPU_OPTIMIZATION=GENERIC`
2. **Library linking errors:** Use bundled libraries (default)
3. **Compiler compatibility:** Use supported compiler versions
4. **SIMD instruction errors:** CPU doesn't support detected SIMD (use GENERIC)
5. **Cross-compilation issues:** Specify target architecture explicitly

### Performance Issues  
```bash
# Check if optimizations are applied
cmake -LA . | grep -E "(CMAKE_CXX_FLAGS|HD_CPU)"

# Verify SIMD detection
cmake --build . --verbose | grep -E "(avx|neon|sse)"

# Compare performance with different optimizations
time ./your_app  # Run with different HD_CPU_OPTIMIZATION values
```

## Platform-Specific Notes

### Windows (Visual Studio)
```cmd
# Configure for Visual Studio 2022
cmake -G "Visual Studio 17 2022" -DCMAKE_BUILD_TYPE=Release ..

# Build
cmake --build . --config Release
```

### Linux (GCC/Clang)  
```bash
# Configure with specific compiler
cmake -DCMAKE_CXX_COMPILER=g++-11 -DCMAKE_BUILD_TYPE=Release ..

# Build with parallel jobs
cmake --build . -- -j$(nproc)
```

### macOS (Xcode/Clang)
```bash
# Configure for Xcode
cmake -G Xcode -DCMAKE_BUILD_TYPE=Release ..

# Build
cmake --build . --config Release

# Apple Silicon optimized build
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=ARM \
    -DCMAKE_OSX_ARCHITECTURES=arm64 ..
```

### ARM Cross-Compilation
```bash
# Raspberry Pi 4 (ARM64)
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=ARM \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ ..

# Android NDK
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=ARM \
    -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
    -DANDROID_ABI=arm64-v8a ..
```

## Performance Optimization Tips

### Compiler-Specific Optimizations
```bash
# GCC with maximum optimization
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=AUTO \
    -DCMAKE_CXX_FLAGS="-O3 -march=native -mtune=native" ..

# Clang with profile-guided optimization (requires training data)
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=AUTO \
    -DCMAKE_CXX_FLAGS="-O3 -fprofile-instr-generate" ..
```

### Memory and Threading
```bash
# Enable OpenMP for parallel processing (if available)
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=AUTO \
    -DCMAKE_CXX_FLAGS="-fopenmp" ..

# Large memory systems (64GB+ RAM)
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=AUTO \
    -DCMAKE_CXX_FLAGS="-DLARGE_MEMORY_SYSTEM" ..
```

### Testing and Validation
```bash
# Enable and build tests
cmake -DCMAKE_BUILD_TYPE=Release \
    -DHD_CPU_OPTIMIZATION=AUTO \
    -DBUILD_TESTING=ON ..
cmake --build . --config Release

# Run all tests
ctest --output-on-failure

# Run specific tests
ctest -R version_system_test
ctest -R toml_structure_test
```

---

**For comprehensive CPU optimization details, see [`CPU_OPTIMIZATION_GUIDE.md`](CPU_OPTIMIZATION_GUIDE.md)**

**For TOML configuration, see [`TOML_CONFIGURATION_GUIDE.md`](TOML_CONFIGURATION_GUIDE.md)**
