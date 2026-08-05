#pragma once
#include <map>
#include <string>
#include <vector>

namespace calib {

// Shared command-line parsing for all CalibrationApp tools.
//
// Flags are stored generically in a multimap (key = flag name without the
// leading "--"), so the same parser serves every tool and new flags need no
// parser changes. Each tool just reads the keys it cares about and ignores the
// rest. Recognised conventions:
//
//   --mjs <file.mjs>            session manifest file; the session directory is
//                               its parent folder (parent_path)
//   --camera_dir <dir>          directory of CAMERA_0 images
//   --laz <a.laz> [b.laz ...]   one or more point clouds (.laz / .las). May be
//                               repeated; consecutive non-flag tokens after a
//                               --laz are all taken as clouds.
//   -h, --help                  print usage and exit
//
// A flag may take several values (each consecutive non-flag token becomes its
// own multimap entry) or none (stored once with an empty value). Tokens that
// don't follow a flag are collected into `positional`, preserving the old
// extension/drag-and-drop behaviour.
struct CliArgs {
    std::multimap<std::string, std::string> opts;        // flag -> value(s)
    std::vector<std::string>                positional;  // non-flag arguments, in order

    bool        help  = false;  // -h / --help was given
    bool        valid = true;   // false on a malformed argument
    std::string error;          // message describing why valid == false

    // True if the flag was present at all (even with an empty value).
    bool has(const std::string& key) const { return opts.find(key) != opts.end(); }

    // First value for `key`, or `def` if absent.
    std::string get(const std::string& key, const std::string& def = {}) const {
        auto it = opts.find(key);
        return it == opts.end() ? def : it->second;
    }

    // All values for `key`, in the order given on the command line.
    std::vector<std::string> getAll(const std::string& key) const {
        std::vector<std::string> v;
        auto range = opts.equal_range(key);
        for (auto it = range.first; it != range.second; ++it) v.push_back(it->second);
        return v;
    }
};

// Parse argv. Never terminates the process — the caller inspects `help` and
// `valid` and decides what to do.
CliArgs parseArgs(int argc, char* argv[]);

// Pre-formatted help lines for the shared flags, so every tool describes the
// same flag the same way. An app passes the subset it actually honours to
// printUsage(); the -h/--help line is always added automatically.
namespace cliopt {
inline constexpr const char* MJS =
    "  --mjs <file.mjs>             session manifest file; the session\n"
    "                               directory is its parent folder";
inline constexpr const char* CAMERA_DIR =
    "  --camera_dir <dir>           directory of CAMERA_0 images";
inline constexpr const char* CALIB =
    "  --calib <file.json>          calibration file (intrinsic + extrinsic)";
inline constexpr const char* LAZ =
    "  --laz <a.laz> [b.laz ...]    one or more point clouds (.laz/.las); may repeat";
}  // namespace cliopt

// Print usage for `appName` listing only `options` (e.g. {cliopt::MJS, ...}).
// `desc` is a one-line summary of the tool. Goes to stdout, or stderr when
// reporting an error (toStderr = true).
void printUsage(const char* appName, const char* desc,
                const std::vector<std::string>& options, bool toStderr = false);

}  // namespace calib