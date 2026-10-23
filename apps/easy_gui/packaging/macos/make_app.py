#!/usr/bin/env python3
"""Builds "Easy Forest HDMapping.app" from a build/bin directory.

Usage: make_app.py <build bin dir> <output dir> [--version X.Y.Z] [--min-macos 13.0]

Layout:
    Contents/MacOS/easy_gui                 main executable (CFBundleExecutable)
    Contents/MacOS/lidar_odometry_step_1    started by easy_gui as a child process from its own directory
    Contents/Frameworks/*.dylib             every non-system dylib either executable needs, recursively

Every bundled Mach-O refers to its bundled dependencies as @rpath/<name>; the executables get the single rpath
@executable_path/../Frameworks and the dylibs @loader_path, so nothing points back into the build machine. The bundle
is then re-signed ad hoc (path rewriting invalidates the linker's signature, and arm64 refuses to run unsigned code)
and checked: every bundled binary must have minos <= --min-macos and may only reference the bundle, /usr/lib and
/System. Any violation fails the script, so a CI build cannot ship a bundle that does not start on the target macOS.
"""

import argparse
import os
import plistlib
import re
import shutil
import subprocess
import sys
from pathlib import Path

APP_NAME = "Easy Forest HDMapping"
BUNDLE_ID = "io.github.martinmokros.easyforesthdmapping"
EXECUTABLES = ["easy_gui", "lidar_odometry_step_1"]
SYSTEM_PREFIXES = ("/usr/lib/", "/System/")


def run(*args):
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def is_macho(path):
    with open(path, "rb") as f:
        magic = f.read(4)
    return magic in (b"\xcf\xfa\xed\xfe", b"\xce\xfa\xed\xfe", b"\xca\xfe\xba\xbe")


def load_commands(path):
    """(dependencies, rpaths, dylib id, minimum macOS) from otool -l."""
    deps, rpaths, dylib_id, minos = [], [], None, None
    cmd = None
    for line in run("otool", "-l", str(path)).splitlines():
        line = line.strip()
        if line.startswith("cmd "):
            cmd = line.split()[1]
        elif line.startswith("name ") and cmd in ("LC_LOAD_DYLIB", "LC_LOAD_WEAK_DYLIB", "LC_REEXPORT_DYLIB"):
            deps.append(line[5:].rsplit(" (offset", 1)[0])
        elif line.startswith("name ") and cmd == "LC_ID_DYLIB":
            dylib_id = line[5:].rsplit(" (offset", 1)[0]
        elif line.startswith("path ") and cmd == "LC_RPATH":
            rpaths.append(line[5:].rsplit(" (offset", 1)[0])
        elif line.startswith("minos ") and cmd == "LC_BUILD_VERSION":
            minos = line.split()[1]
        elif line.startswith("version ") and cmd == "LC_VERSION_MIN_MACOSX":
            minos = line.split()[1]
    return deps, rpaths, dylib_id, minos


def expand(path, loader, executable):
    return Path(path.replace("@loader_path", str(loader.parent)).replace("@executable_path", str(executable.parent)))


def resolve(dep, macho, rpaths, executable):
    """Where the build-tree binary `macho` really loads `dep` from."""
    if dep.startswith("@rpath/"):
        for rpath in rpaths:
            candidate = expand(rpath, macho, executable) / dep[len("@rpath/"):]
            if candidate.exists():
                return candidate.resolve()
        return None
    candidate = expand(dep, macho, executable)
    return candidate.resolve() if candidate.exists() else None


def version_tuple(v):
    return tuple(int(x) for x in v.split("."))


def bundle(bin_dir, out_dir, version, min_macos):
    app = out_dir / f"{APP_NAME}.app"
    if app.exists():
        shutil.rmtree(app)
    macos_dir = app / "Contents" / "MacOS"
    frameworks = app / "Contents" / "Frameworks"
    resources = app / "Contents" / "Resources"
    for d in (macos_dir, frameworks, resources):
        d.mkdir(parents=True)

    # Source binary -> bundled copy, walked breadth first so each dylib is resolved with the rpaths of the build-tree
    # binary that loads it.
    queue = []
    for name in EXECUTABLES:
        src = bin_dir / name
        if not src.is_file():
            sys.exit(f"error: {src} not found")
        dst = macos_dir / name
        shutil.copy2(src, dst)
        queue.append((src, dst, src))

    bundled = {}  # resolved source path -> bundled name
    while queue:
        src, dst, executable = queue.pop(0)
        deps, rpaths, _, _ = load_commands(src)
        for dep in deps:
            if dep.startswith(SYSTEM_PREFIXES):
                continue
            real = resolve(dep, src, rpaths, executable)
            if real is None:
                sys.exit(f"error: {src.name} needs {dep}, which cannot be found")
            name = Path(dep).name
            if real not in bundled:
                if any(n == name for n in bundled.values()):
                    sys.exit(f"error: two different dylibs are both called {name}")
                bundled[real] = name
                lib_dst = frameworks / name
                shutil.copy2(real, lib_dst)
                lib_dst.chmod(0o755)
                queue.append((real, lib_dst, executable))
                print(f"  bundling {real} as Frameworks/{name}")
            if dep != f"@rpath/{bundled[real]}":
                run("install_name_tool", "-change", dep, f"@rpath/{bundled[real]}", str(dst))
        for rpath in rpaths:
            run("install_name_tool", "-delete_rpath", rpath, str(dst))
        new_rpath = "@executable_path/../Frameworks" if dst.parent == macos_dir else "@loader_path"
        run("install_name_tool", "-add_rpath", new_rpath, str(dst))
        if dst.parent == frameworks:
            run("install_name_tool", "-id", f"@rpath/{dst.name}", str(dst))

    with open(app / "Contents" / "Info.plist", "wb") as f:
        plistlib.dump(
            {
                "CFBundleDevelopmentRegion": "en",
                "CFBundleDisplayName": APP_NAME,
                "CFBundleName": APP_NAME,
                "CFBundleExecutable": "easy_gui",
                "CFBundleIdentifier": BUNDLE_ID,
                "CFBundleInfoDictionaryVersion": "6.0",
                "CFBundlePackageType": "APPL",
                "CFBundleShortVersionString": version,
                "CFBundleVersion": version,
                "LSMinimumSystemVersion": min_macos,
                "LSApplicationCategoryType": "public.app-category.utilities",
                "NSHighResolutionCapable": True,
                "NSHumanReadableCopyright": "MIT License. Built on HDMapping by Janusz Będkowski and the MapsHD contributors.",
            },
            f,
        )

    # Only after all rewriting: install_name_tool invalidates signatures.
    run("codesign", "--force", "--deep", "-s", "-", str(app))
    return app


def verify(app, min_macos):
    errors = []
    machos = [p for p in app.rglob("*") if p.is_file() and not p.is_symlink() and is_macho(p)]
    frameworks = app / "Contents" / "Frameworks"
    print(f"Checking {len(machos)} Mach-O files (minos <= {min_macos}):")
    for p in sorted(machos):
        deps, rpaths, _, minos = load_commands(p)
        rel = p.relative_to(app)
        print(f"  {rel}: minos {minos}")
        if minos is None:
            errors.append(f"{rel}: no minimum macOS version in the load commands")
        elif version_tuple(minos) > version_tuple(min_macos):
            errors.append(f"{rel}: minos {minos} > {min_macos}")
        for rpath in rpaths:
            if not rpath.startswith(("@executable_path", "@loader_path")):
                errors.append(f"{rel}: rpath {rpath} points outside the bundle")
        for dep in deps:
            if dep.startswith(SYSTEM_PREFIXES):
                continue
            if not dep.startswith("@rpath/") or not (frameworks / dep[len("@rpath/"):]).is_file():
                errors.append(f"{rel}: dependency {dep} is not in Contents/Frameworks")
    try:
        run("codesign", "--verify", "--deep", "--strict", str(app))
    except subprocess.CalledProcessError as e:
        errors.append(f"codesign --verify failed: {e.stderr.strip()}")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bin_dir", type=Path)
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--version", default="0.0.0")
    parser.add_argument("--min-macos", default="13.0")
    args = parser.parse_args()

    version = re.sub(r"^[^0-9]*", "", args.version) or "0.0.0"
    args.out_dir.mkdir(parents=True, exist_ok=True)
    app = bundle(args.bin_dir.resolve(), args.out_dir.resolve(), version, args.min_macos)
    errors = verify(app, args.min_macos)
    if errors:
        print("\nBundle check FAILED:", file=sys.stderr)
        for e in errors:
            print(f"  {e}", file=sys.stderr)
        sys.exit(1)
    print(f"\n{app} OK")


if __name__ == "__main__":
    main()
