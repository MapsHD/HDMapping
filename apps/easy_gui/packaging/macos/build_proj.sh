#!/usr/bin/env bash
# Builds a static PROJ for the Easy Forest HDMapping macOS release.
#
# Usage: build_proj.sh <install prefix> [work dir]
#
# Why: core_math links PROJ::proj (core/src/gnss.cpp, GNSS georeferencing, which Easy GUI never calls), so every
# build needs one. Homebrew's libproj is built for the runner's own macOS version (and pulls in libtiff/curl/sqlite
# dylibs built the same way), so a bundle using it would not start on older macOS. A static PROJ built here for the
# deployment target leaves nothing to bundle: only the system libsqlite3 and libc++ are linked dynamically.
# TIFF (grids) and curl (network grids) are off: neither is needed to compile/link core_math.
set -euo pipefail

PROJ_VERSION=9.6.2
PROJ_SHA256=53d0cafaee3bb2390264a38668ed31d90787de05e71378ad7a8f35bb34c575d1
DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-13.0}"

prefix="${1:?usage: build_proj.sh <install prefix> [work dir]}"
work="${2:-$(mktemp -d)}"
mkdir -p "$work"
prefix="$(cd "$(dirname "$prefix")" && pwd)/$(basename "$prefix")"

tarball="$work/proj-$PROJ_VERSION.tar.gz"
if [ ! -f "$tarball" ]; then
    curl -sSLf -o "$tarball" "https://github.com/OSGeo/PROJ/releases/download/$PROJ_VERSION/proj-$PROJ_VERSION.tar.gz"
fi
echo "$PROJ_SHA256  $tarball" | shasum -a 256 -c -

rm -rf "$work/proj-$PROJ_VERSION" "$work/build"
tar -xzf "$tarball" -C "$work"

cmake -S "$work/proj-$PROJ_VERSION" -B "$work/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET" \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_APPS=OFF \
    -DENABLE_TIFF=OFF \
    -DENABLE_CURL=OFF \
    -DEMBED_PROJ_DATA_PATH=OFF
cmake --build "$work/build" --config Release -j"$(sysctl -n hw.ncpu)"
cmake --install "$work/build" --config Release

echo "PROJ $PROJ_VERSION installed to $prefix (macOS $DEPLOYMENT_TARGET, static)"
