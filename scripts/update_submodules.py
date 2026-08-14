#!/usr/bin/env python3

import os
import subprocess
import sys


# ============================================================================
# Configuration
# ============================================================================

SUBMODULES_DIR = "3rdparty"


# ============================================================================
# Paths
# ============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
SUBMODULES_ROOT = os.path.join(REPO_ROOT, SUBMODULES_DIR)


# ============================================================================
# Git helpers
# ============================================================================

def run_git(args, cwd, quiet=False):
    result = subprocess.run(
        ["git"] + args,
        cwd=cwd,
        text=True,
        capture_output=True,
    )

    if result.returncode != 0:
        if not quiet:
            print("ERROR: git " + " ".join(args))

            if result.stderr:
                print(result.stderr.strip())

        return None

    return result.stdout.strip()


# ============================================================================
# Submodules
# ============================================================================

def get_submodules():
    output = run_git(
        [
            "config",
            "--file",
            ".gitmodules",
            "--get-regexp",
            r"^submodule\..*\.path$",
        ],
        REPO_ROOT,
    )

    if output is None:
        return []

    submodules = []

    for line in output.splitlines():
        key, path = line.split(None, 1)

        # Extract submodule name from:
        # submodule.NAME.path
        name = key[len("submodule."):-len(".path")]

        path = os.path.normpath(path)

        # Only include submodules located inside SUBMODULES_DIR.
        if (
            path == SUBMODULES_DIR
            or path.startswith(SUBMODULES_DIR + os.sep)
        ):
            submodules.append((name, path))

    return submodules


def get_branch(submodule_name):
    return run_git(
        [
            "config",
            "--file",
            ".gitmodules",
            "--get",
            "submodule." + submodule_name + ".branch",
        ],
        REPO_ROOT,
        quiet=True,
    )


# ============================================================================
# Update
# ============================================================================

def update_submodule(submodule_name, submodule_path):
    submodule_dir = os.path.join(REPO_ROOT, submodule_path)

    print()
    print("=" * 70)
    print("Updating:", submodule_path)
    print("=" * 70)

    branch = get_branch(submodule_name)

    if branch is None:
        print("ERROR: no branch configured in .gitmodules")
        print("Submodule:", submodule_name)
        print("Skipping:", submodule_path)

        return False, None, False, "No branch configured"

    print("Pinned branch:", branch)

    # ------------------------------------------------------------------------
    # Initialize missing submodule
    # ------------------------------------------------------------------------

    if not os.path.isdir(submodule_dir):
        result = subprocess.run(
            [
                "git",
                "submodule",
                "update",
                "--init",
                "--",
                submodule_path,
            ],
            cwd=REPO_ROOT,
        )

        if result.returncode != 0:
            print("ERROR: failed to initialize submodule")

            return False, branch, False, "Initialization failed"

    # ------------------------------------------------------------------------
    # Fetch
    # ------------------------------------------------------------------------

    result = subprocess.run(
        ["git", "fetch", "origin"],
        cwd=submodule_dir,
    )

    if result.returncode != 0:
        print("ERROR: fetch failed")

        return False, branch, False, "Fetch failed"

    # ------------------------------------------------------------------------
    # Get current branch
    # ------------------------------------------------------------------------

    current_branch = run_git(
        ["symbolic-ref", "--short", "HEAD"],
        submodule_dir,
        quiet=True,
    )

    # ------------------------------------------------------------------------
    # Switch to pinned branch if necessary
    # ------------------------------------------------------------------------

    if current_branch != branch:
        if current_branch is None:
            print("Switching: HEAD ->", branch)
        else:
            print("Switching:", current_branch, "->", branch)

        result = subprocess.run(
            ["git", "checkout", branch],
            cwd=submodule_dir,
        )

        if result.returncode != 0:
            # Local branch does not exist.
            result = subprocess.run(
                [
                    "git",
                    "checkout",
                    "-b",
                    branch,
                    "--track",
                    "origin/" + branch,
                ],
                cwd=submodule_dir,
            )

            if result.returncode != 0:
                print("ERROR: checkout failed")

                return False, branch, False, "Checkout failed"

    # ------------------------------------------------------------------------
    # Get local and remote commits BEFORE pull
    # ------------------------------------------------------------------------

    before_commit = run_git(
        ["rev-parse", "HEAD"],
        submodule_dir,
        quiet=True,
    )

    remote_commit = run_git(
        ["rev-parse", "origin/" + branch],
        submodule_dir,
        quiet=True,
    )

    if before_commit is None:
        print("ERROR: could not determine local commit")

        return False, branch, False, "Could not determine local commit"

    if remote_commit is None:
        print("ERROR: could not determine remote commit")

        return False, branch, False, "Could not determine remote commit"

    # ------------------------------------------------------------------------
    # Pull
    # ------------------------------------------------------------------------

    result = subprocess.run(
        [
            "git",
            "pull",
            "--ff-only",
            "origin",
            branch,
        ],
        cwd=submodule_dir,
    )

    if result.returncode != 0:
        print("ERROR: pull failed")

        return False, branch, False, "Pull failed"

    # ------------------------------------------------------------------------
    # Get final commit
    # ------------------------------------------------------------------------

    after_commit = run_git(
        ["rev-parse", "HEAD"],
        submodule_dir,
        quiet=True,
    )

    if after_commit is None:
        print("ERROR: could not determine final commit")

        return False, branch, False, "Could not determine final commit"

    # ------------------------------------------------------------------------
    # Determine whether anything was actually pulled
    # ------------------------------------------------------------------------

    updated = before_commit != after_commit

    if updated:
        print("Updated successfully:", submodule_path)

        return True, branch, True, "Changes pulled"

    print("Already up to date:", submodule_path)

    return True, branch, False, "Already up to date"


# ============================================================================
# Main
# ============================================================================

def main():
    if not os.path.isdir(os.path.join(REPO_ROOT, ".git")):
        print("ERROR: repository root not found")
        sys.exit(1)

    if not os.path.isdir(SUBMODULES_ROOT):
        print(
            "ERROR: submodule directory does not exist:",
            SUBMODULES_ROOT,
        )
        sys.exit(1)

    submodules = get_submodules()

    if not submodules:
        print(
            "No submodules found under:",
            SUBMODULES_DIR,
        )
        return

    print("Found submodules:")

    for submodule_name, submodule_path in submodules:
        print("   ", submodule_path)

    success = True
    results = []

    for submodule_name, submodule_path in submodules:
        result, branch, updated, reason = update_submodule(
            submodule_name,
            submodule_path,
        )

        if not result:
            success = False

        results.append(
            (
                submodule_path,
                branch if branch is not None else "-",
                updated,
                reason,
            )
        )

    # ------------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------------

    print()
    print("=" * 105)
    print("SUBMODULE UPDATE SUMMARY")
    print("=" * 105)

    print(
        "{:<40} {:<20} {:<10} {}".format(
            "Submodule",
            "Branch",
            "Updated",
            "Reason",
        )
    )

    print("-" * 105)

    for submodule_path, branch, updated, reason in results:
        print(
            "{:<40} {:<20} {:<10} {}".format(
                submodule_path,
                branch,
                "YES" if updated else "NO",
                reason,
            )
        )

    print("=" * 105)

    if success:
        print("All submodules updated successfully.")
        sys.exit(0)

    print("Some submodules failed to update.")
    sys.exit(1)


if __name__ == "__main__":
    main()