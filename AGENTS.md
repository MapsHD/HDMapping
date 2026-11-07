# AGENTS.md

Guidance for coding agents (and humans) working on HDMapping. The rules come from maintainer review comments; the
number in brackets is the PR where a rule was raised.

## Project

HDMapping is the processing toolset for the Mandeye mobile mapping system. C++20, CMake >= 4.0. For supported
platforms, see "Platforms and portability" below.

- `core/`, `calib_core/`: shared libraries (point clouds, registration, raylib rendering in the `core_raylib` target).
- `shared/include/HDMapping/`: header-only utilities (`Assert.h` with `HDMAPPING_ASSERT`, `PoseInterpolation.h`,
  `SessionPaths.h`).
- `apps/`: the tools. Main workflow: `mandeye_raw_data_viewer`, `lidar_odometry_step_1`, `multi_view_tls_registration`
  (step 2), `multi_session_registration` (step 3). The `*_legacy` apps are the GLUT versions kept next to the raylib
  ports. New rendering work goes into the raylib code.
- `3rdparty/`: git submodules built from source. `3rdpartyBinary/`: prebuilt OpenCV and PROJ. Both are wired up in
  `cmake/dependencies.cmake`.
- `scripts/`: developer tooling, documented in `scripts/README.md`.
- User documentation lives in the GitHub wiki, not in the repo.

## Build and check

```bash
git submodule update --init --recursive
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_UTILITY_APPLICATION=ON -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
python3 scripts/run_check_clang_format.py  # needs clang-format 21 (same as CI); reformats files in place
```

Ubuntu packages are listed in `ubuntu-24.04-apt-requirements.sh`. CI (`.github/workflows/`) builds Linux, Windows,
macOS, the Debian package and the Python bindings. It fails on any clang-format difference.

## Scope of a change

- One concern per PR. A bug fix and a feature or UX change go in separate PRs, even when they were found together.
  (#590)
- A bug found along the way (UB, race, crash) gets an issue and a minimal fix of a few lines, such as a mutex. Don't
  rewrite the code around it. (#590)
- Keep diffs reviewable. Agent-written code is fine, but a large diff that mixes fixes, refactors, tests and UX will
  be sent back to be split. (#590)
- Performance work must not change results. Give before/after timings. (#5)

## Existing workflows

- Changes must not break existing workflows. The processing chain (raw data viewer, step 1, step 2, step 3; see
  `images/workflow.png`) must keep working end to end.
- Files written by earlier versions must still load: sessions (`.mjs`), projects (`.mjp`), calibrations (`.mjc`) and
  step outputs. If you change a format, keep reading the old one.
- Existing command-line options and parameters keep their names and meaning. Add new ones rather than renaming or
  repurposing old ones.
- Call out in the PR description any change users will notice: defaults, output files, results.

## Reuse before adding

- Search `core/` and `shared/` before adding a struct, helper or class. Don't duplicate one that already exists. (#590)
- When a library already has a facility, use it instead of writing your own: raylib's mesh and buffer drawing,
  Eigen's `Map` and `cast`. (#590, #192)
- A new class with its own responsibility gets its own `.h`/`.cpp` pair and a short doc comment. Don't append it to a
  large existing `.cpp`. (#555)

## Platforms and portability

- **Tier 1: Latest Windows 11.** Every change must build and work here. Test user-facing changes on Windows before merging.
  (#576)
- **Tier 2: macOS and Ubuntu 24.04.** Every change must build and work here too. CI builds and tests both on every PR.
- **Tier 3: everything else** (other Linux distributions and versions, other architectures, Windows 10). Best effort. Fixes are
  welcome as long as they don't affect tier 1 or tier 2.
- Features that work on only one platform are not merged. (#97, #571, #576)
- Paths: use `std::filesystem::path` and `operator/`, never string concatenation with `"/"`. (#555)
- Optional or debug behaviour goes behind a CMake option or preprocessor flag. Don't leave code commented out. (#64)

## Build and third-party libraries

- The build is self-contained. After `git clone --recursive` and installing the documented system packages (apt on
  Linux, Homebrew on macOS), `cmake -B build && cmake --build build` works on every tier 1 and tier 2 platform. A
  change must not add any other manual step, such as running a script or downloading a file by hand.
- Third-party libraries are built from source as `3rdparty/` submodules wherever possible.
- If a library can't be built from source, CMake downloads prebuilt binaries and checks a pinned SHA256 (see
  `3rdpartyBinary/`). They come from the library's official release (e.g. OpenCV for Windows) or, if upstream ships
  none, from a CI workflow in a repository of the MapsHD GitHub organization (e.g. PROJ for Windows from
  `MapsHD/proj_win_deploy`). Binaries built by hand are not accepted.
- System packages from the platform's package manager (apt, Homebrew) are fine. A new one goes into
  `ubuntu-24.04-apt-requirements.sh`, the CI workflows and the build docs in the same PR.
- Every tool in the toolset links against the same version of each third-party library. Don't give one tool its own
  copy or its own way of getting a library (e.g. one tool building its own PROJ while the others use the shared one).
  Changing a library for one tool means changing it for all of them. (#571, #97)
- Don't move submodule commits by accident. Check `git diff --submodule` before committing. (#247)

## One product, one release

- All tools share the version in the top-level `CMakeLists.txt` (`project(hd-mapping VERSION ...)`) and are released
  together. No tool gets its own version, CI workflow, release script or packaging script. (#571)
- A toolchain bump (CMake, compiler) updates the CI workflows and the build docs in the same PR. (#216)
- Packaging is done in one place per platform: `deploy_mandeye.bat` (Windows), `macos_bundle_toolkit/` (macOS) and
  CPack (Debian).

### Adding a new executable

Nothing fails if you miss one of these steps:

1. `add_subdirectory(apps/<tool>)` in the top-level `CMakeLists.txt`, under the right `BUILD_WITH_*` option. Without
   it the tool isn't built.
2. `hdmapping_install_app(<target>)` in the tool's `CMakeLists.txt`. Without it the tool is missing from the install
   and the Debian package.
3. The `EXPECTED` list in `deploy_mandeye.bat`. Without it the Windows deploy doesn't warn when the tool stops being
   built.

## Tests

- Unit tests (doctest, enabled with `BUILD_TESTING`) are for logic with a clear contract: math, parsing,
  interpolation, file formats.
- Don't add tests for things you can check by looking at the output (rendering, GUI, UX). They are a maintenance
  burden. A test must not encode an agent's reasoning steps. (#590)

## Docs and comments

- No design docs or agent notes in the source tree. Lasting guidance for agents belongs in this file, trimmed by a
  human. (#571)
- Every developer tool gets a line in `scripts/README.md`. (#486)
- Doc comments use `//!` Doxygen on declarations. Keep them short.

## CI etiquette

- A commit that touches only docs, images or the README gets `[skip ci]` in its message. (#437, #304)
