## Quick Start (Ubuntu / Ubuntu 24.04 @ WSL2)

### CMake 4.0.0 or Higher

This project requires **CMake 4.0.0** or higher. If you don't have it installed, you can download it from:

**Official CMake Downloads:**

- **Linux/macOS/Windows:** https://cmake.org/download/
- **GitHub Releases (Linux binaries):** https://github.com/Kitware/CMake/releases/tag/v4.0.0

**Installation on Linux:**

```bash
wget https://github.com/Kitware/CMake/releases/download/v4.0.0/cmake-4.0.0-linux-x86_64.sh
sudo sh cmake-4.0.0-linux-x86_64.sh --skip-license --prefix=/usr/local
cmake --version
```

```bash
git clone --recursive https://github.com/MapsHD/HDMapping.git
cd HDMapping
./ubuntu-24.04-apt-requirements.sh

# Auto-optimized build (detects your CPU automatically)
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j
```

**Laptops with hybrid NVIDIA/Intel graphics:** the raylib-based apps (e.g. `multi_view_tls_registration_step_2`) may default to the integrated GPU even with `prime-select nvidia` set, since PRIME's on-demand/offload mode is a per-launch choice, not a system default. Force the discrete NVIDIA GPU with:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ./build/bin/multi_view_tls_registration_step_2
```

If the `nvidia-prime` package is installed, `prime-run` wraps the same env vars:

```bash
prime-run ./build/bin/multi_view_tls_registration_step_2
```

## Quick Start (macOS)

**Prerequisites:**

1. Install **XCode Command Line Tools**: `xcode-select --install`
2. Install **Homebrew** from [brew.sh](https://brew.sh)
3. Install dependencies:

```bash
brew install cmake opencv
```

**Build:**

```bash
git clone --recursive https://github.com/MapsHD/HDMapping.git
cd HDMapping

# Configure with Cocoa support (native macOS windowing)
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DFREEGLUT_COCOA=ON

# Build (auto-detects number of cores)
cmake --build build -j$(sysctl -n hw.ncpu)
```

# Building commands

## Requirements

**Installation on Windows:**

- Download the installer from https://cmake.org/download/
- Run the installer and follow the instructions
- Ensure CMake is added to your system PATH

### clang-format

**Instalation on Linux:**

```bash
sudo apt install clang-format
```

**Installation on Windows**

1. Go to offical llvm-project GitHub [releases page](https://github.com/llvm/llvm-project/releases)
2. Download Windows x64 installer (for example version [21.1.8](https://github.com/llvm/llvm-project/releases/download/llvmorg-21.1.8/LLVM-21.1.8-win64.exe)) - browser might flag exe file malware in that case mark it as "Keep it"
3. Run installer exe
4. In installation program mark "Add LLVM to the system PATH for all users" or "Add LLVM to the system PATH for all current user"
5. Restart any terminal or IDE that you want to use clang-format in so it reloads paths from ENV

**Verification**

To verify that clang-format is installed run:
Verify installation by running:

```bash
clang-format --version
```

**Formating codebase**

In order to format code base run:

```bash
python3 run_clang_fromat.py
```

_Note that PRs without formatting might be rejected from merging_

## Quick Start (Windows)

```bash
git clone --recursive https://github.com/MapsHD/HDMapping.git
cd HDMapping

# Auto-optimized build (detects your CPU automatically)
cmake  -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

## WSL2 GUI

On WSL2 to enable file dialogs in GUI applications you need to install one of the packages used by portable-file-dialogs listed [here](https://github.com/samhocevar/portable-file-dialogs/blob/c12ea8c9a727f5320a2b4570aee863bbede2a204/portable-file-dialogs.h#L539C1-L542C57).

For example on WSL2 Ubuntu-24.04 following package is required to run GUI applications:

```bash
sudo apt install zenity
```
# Profiling

You can use multiple backends to profile the code (UTL, Tracy-Profiler)

- Using Tracy-Profiler requires to install / build from source Tracy-Profiler from https://github.com/wolfpld/tracy/releases/tag/v0.13.1.
- Build and run the project with `cmake .. -DCMAKE_BUILD_TYPE=ReleaseWithDebInfo -DHDMAPPING_PROFILER=TRACY`
- Open Tracy Profiler and run e.g. `lidar_odometry_step_1`. Tracy-Profiler should recongnize the code and show the results.
- 
To use UTL profiler build and run the project with `cmake .. -DCMAKE_BUILD_TYPE=ReleaseWithDebInfo -DHDMAPPING_PROFILER=UTL`

# Building Debian package.

The standard build contains all necessary libraries compiled with project.
This approach allows smooth build on Windows platform and guarantee predictable experience.
If you want to build Debian package, you can depends on system-provided libraries:
Before build install 3rd party libraries:

```
sudo apt-get install freeglut3-dev libeigen3-dev liblaszip-dev libopencv-dev
```

Next build Debian package:

```
cmake .. -DBUILD_WITH_BUNDLED_FREEGLUT=0 -DBUILD_WITH_BUNDLED_EIGEN=0 -DBUILD_WITH_BUNDLED_LIBLASZIP=0 -DCMAKE_BUILD_TYPE=Release
make -j16
make package
```

To install package :

```
sudo dpkg -i hd_mapping-0.*.*-Linux.deb
```
