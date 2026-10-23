#!/usr/bin/env python3
"""Builds the Easy Forest HDMapping Windows folder and zip from a build/bin/Release directory.

Usage: make_zip.py <build bin dir> <output folder> <output .zip> [--search-dir DIR ...]

The folder holds easy_gui.exe, lidar_odometry_step_1.exe (started by easy_gui from its own directory) and every DLL
they need, found recursively with `dumpbin /dependents`. The MSVC runtime (msvcp140, vcruntime140, ...) is copied
app-locally from Visual Studio's redist directory: the build runner has it installed system-wide, a user's PC may not.
Everything else must be part of Windows (api-ms-win-*, or present in System32 and not a VC runtime DLL); a DLL that
can be found neither in the build nor in --search-dir fails the script. The zip contains the files at its top level
(README: extract into a folder, double-click easy_gui.exe).
"""

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

EXECUTABLES = ["easy_gui.exe", "lidar_odometry_step_1.exe"]
VC_RUNTIME = re.compile(r"^(msvcp\d+.*|vcruntime\d+.*|concrt\d+|vcomp\d+|vccorlib\d+)\.dll$", re.IGNORECASE)
SYSTEM32 = Path(os.environ.get("SystemRoot", r"C:\Windows")) / "System32"


def vswhere(*args):
    exe = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
    return subprocess.run([str(exe), "-latest", "-products", "*", *args], check=True, capture_output=True, text=True).stdout.strip()


def find_tools():
    vs = Path(vswhere("-property", "installationPath"))
    dumpbin = sorted(glob.glob(str(vs / "VC/Tools/MSVC/*/bin/Hostx64/x64/dumpbin.exe")))
    redist = sorted(glob.glob(str(vs / "VC/Redist/MSVC/*/x64")))
    if not dumpbin or not redist:
        sys.exit(f"error: dumpbin or the VC redist directory not found under {vs}")
    # Highest version wins; the redist folder holds Microsoft.VC14x.CRT (and .OpenMP) subfolders.
    crt_dirs = [Path(d) for d in glob.glob(str(Path(redist[-1]) / "Microsoft.VC*.CRT"))]
    crt_dirs += [Path(d) for d in glob.glob(str(Path(redist[-1]) / "Microsoft.VC*.OpenMP"))]
    return Path(dumpbin[-1]), crt_dirs


def dependents(dumpbin, binary):
    out = subprocess.run([str(dumpbin), "/nologo", "/dependents", str(binary)], check=True, capture_output=True, text=True).stdout
    return [line.strip() for line in out.splitlines() if line.strip().lower().endswith(".dll") and " " not in line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bin_dir", type=Path)
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("zip", type=Path)
    parser.add_argument("--search-dir", type=Path, action="append", default=[])
    args = parser.parse_args()

    dumpbin, crt_dirs = find_tools()
    print(f"dumpbin: {dumpbin}\nVC runtime: {', '.join(map(str, crt_dirs))}")

    if args.out_dir.exists():
        shutil.rmtree(args.out_dir)
    args.out_dir.mkdir(parents=True)

    queue = []
    for name in EXECUTABLES:
        src = args.bin_dir / name
        if not src.is_file():
            sys.exit(f"error: {src} not found")
        shutil.copy2(src, args.out_dir / name)
        queue.append(args.out_dir / name)

    search = [args.bin_dir, *args.search_dir]
    copied, system, errors = set(), set(), []
    while queue:
        binary = queue.pop(0)
        for dll in dependents(dumpbin, binary):
            key = dll.lower()
            if key in copied or key in system:
                continue
            if key.startswith(("api-ms-win-", "ext-ms-")):
                system.add(key)
                continue
            if VC_RUNTIME.match(dll):
                source = next((d / dll for d in crt_dirs if (d / dll).is_file()), None)
            else:
                source = next((d / dll for d in search if (d / dll).is_file()), None)
                if source is None and (SYSTEM32 / dll).is_file():
                    system.add(key)
                    continue
            if source is None:
                errors.append(f"{binary.name} needs {dll}, which is neither in the build, a search dir nor Windows")
                continue
            shutil.copy2(source, args.out_dir / source.name)
            copied.add(key)
            print(f"  {binary.name} -> {dll}  (from {source.parent})")
            queue.append(args.out_dir / source.name)

    # The C++ runtime DLLs load each other dynamically in places (e.g. msvcp140 -> msvcp140_1/_2 for some
    # features), so ship the whole CRT folder rather than only what dumpbin lists.
    for d in crt_dirs:
        if d.name.endswith(".CRT"):
            for dll in d.glob("*.dll"):
                if dll.name.lower() not in copied:
                    shutil.copy2(dll, args.out_dir / dll.name)
                    copied.add(dll.name.lower())

    if errors:
        print("\nPackage check FAILED:", file=sys.stderr)
        for e in errors:
            print(f"  {e}", file=sys.stderr)
        sys.exit(1)

    print(f"\nSystem DLLs (not bundled): {', '.join(sorted(system))}")
    print(f"Folder contents:")
    for f in sorted(args.out_dir.iterdir()):
        print(f"  {f.name:40s} {f.stat().st_size / 1e6:8.2f} MB")

    args.zip.unlink(missing_ok=True)
    with zipfile.ZipFile(args.zip, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for f in sorted(args.out_dir.iterdir()):
            z.write(f, f.name)
    print(f"\n{args.zip}: {args.zip.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
