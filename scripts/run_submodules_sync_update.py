#!/usr/bin/env python3

import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)


def run_git(args):
    result = subprocess.run(
        ["git"] + args,
        cwd=REPO_ROOT,
    )

    if result.returncode != 0:
        print("ERROR: git " + " ".join(args))
        return False

    return True


def main():
    if not os.path.isdir(os.path.join(REPO_ROOT, ".git")):
        print("ERROR: repository root not found")
        sys.exit(1)

    print("Running: git submodule sync")
    if not run_git(["submodule", "sync"]):
        sys.exit(1)

    print()
    print("Running: git submodule update --init --recursive")
    if not run_git(["submodule", "update", "--init", "--recursive"]):
        sys.exit(1)

    print()
    print("Submodules synchronized and initialized successfully.")


if __name__ == "__main__":
    main()
