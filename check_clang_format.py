#!/usr/bin/env python3

import os
import subprocess
import sys

PROJECT_DIRECTORY = os.path.abspath(os.path.dirname(__file__))

FILE_LOCATIONS = [
    os.path.join(PROJECT_DIRECTORY, "core"),
    os.path.join(PROJECT_DIRECTORY, "core_hd_mapping"),
    os.path.join(PROJECT_DIRECTORY, "apps"),
    os.path.join(PROJECT_DIRECTORY, "pybind"),
    os.path.join(PROJECT_DIRECTORY, "shared"),
]

FILE_EXTENSIONS = [
    ".cpp",
    ".hpp",
    ".c",
    ".h",
]


def get_files(input_paths: list[str], extensions: list[str]):
    files = []
    for input_path in input_paths:
        for dirpath, _, filenames in os.walk(input_path):
            for filename in filenames:
                if filename.endswith(tuple(extensions)):
                    files.append(os.path.normpath(os.path.join(dirpath, filename)))
    return files


def run(cmd):
    return subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def main():
    formatted_files = set(get_files(FILE_LOCATIONS, FILE_EXTENSIONS))

    formatted_files = {
        os.path.relpath(path, PROJECT_DIRECTORY)
        for path in formatted_files
    }

    result = run([sys.executable, "run_clang_format.py"])
    if result.returncode != 0:
        print("ERROR: run_clang_format.py failed")
        print(result.stderr)
        sys.exit(result.returncode)

    status = run(["git", "status", "--porcelain"])
    if status.returncode != 0:
        print("ERROR: git status failed")
        print(status.stderr)
        sys.exit(status.returncode)

    changed_files = set()

    for line in status.stdout.splitlines():
        path = line[3:]
        changed_files.add(os.path.normpath(path))

    offending_files = sorted(formatted_files & changed_files)

    if offending_files:
        print("ERROR: clang-format produced changes in the following files:")
        for f in offending_files:
            print(f"  {f}")
        sys.exit(1)

    print("clang-format check passed")


if __name__ == "__main__":
    main()
