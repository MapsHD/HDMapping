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