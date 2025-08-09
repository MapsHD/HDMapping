# CMake Configuration Helper for HDMapping

## Quick Reference

### CPU Optimization Options
```bash
# Option 1: Auto-detect (Recommended)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Option 2: Force AMD optimizations
cmake -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=AMD ..

# Option 3: Force Intel optimizations  
cmake -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=INTEL ..

# Option 4: Generic compatibility
cmake -DCMAKE_BUILD_TYPE=Release -DHD_CPU_OPTIMIZATION=GENERIC ..
```

### Build Type Options
```bash
# Release build (optimized)
cmake -DCMAKE_BUILD_TYPE=Release ..

# Debug build (with debug symbols)
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release with debug info
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
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

### Python Bindings
```bash
# Enable Python bindings
cmake -DCMAKE_BUILD_TYPE=Release -DPYBIND=ON ..
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
| `CMAKE_BUILD_TYPE` | Release | Debug, Release, RelWithDebInfo | Build configuration |
| `HD_CPU_OPTIMIZATION` | AUTO | AUTO, AMD, INTEL, GENERIC | CPU optimization strategy |
| `BUILD_WITH_BUNDLED_FREEGLUT` | ON | ON, OFF | Use bundled FreeGLUT library |
| `BUILD_WITH_BUNDLED_EIGEN` | ON | ON, OFF | Use bundled Eigen library |
| `BUILD_WITH_BUNDLED_LIBLASZIP` | ON | ON, OFF | Use bundled LASzip library |
| `PYBIND` | OFF | ON, OFF | Enable Python bindings |

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
```
