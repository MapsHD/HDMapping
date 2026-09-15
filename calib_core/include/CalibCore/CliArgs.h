#pragma once
#include <map>
#include <string>
#include <vector>

namespace calib {

//! Shared command-line parsing for all CalibrationApp tools.
//!
//! Flags are stored generically in a multimap (key = flag name without the
//! leading "--"), so the same parser serves every tool and new flags need no
//! parser changes. Each tool reads the keys it cares about and ignores the
//! rest. Recognised conventions:
//!
//!       --mjs <file.mjs>            session manifest file; the session
//!                                   directory is its parent folder
//!       --camera_dir <dir>          directory of CAMERA_0 images
//!       --laz <a.laz> [b.laz ...]   one or more point clouds (.laz / .las);
//!                                   may be repeated
//!       -h, --help                  print usage and exit
//!
//! @note A flag may take several values -- each consecutive non-flag token
//!       becomes its own multimap entry -- or none, in which case it is stored
//!       once with an empty value. Tokens that don't follow a flag are
//!       collected into @ref positional, preserving the old
//!       extension/drag-and-drop behaviour.
struct CliArgs {
    //! Flag name (without "--") to value(s).
    std::multimap<std::string, std::string> opts;
    //! Non-flag arguments, in the order given.
    std::vector<std::string>                positional;

    //! -h / --help was given.
    bool        help  = false;
    //! False on a malformed argument; see @ref error.
    bool        valid = true;
    //! Message describing why @ref valid is false.
    std::string error;

    //! Whether the flag was present at all, even with an empty value.
    //! @param key flag name, without the leading "--"
    //! @return true when present
    bool has(const std::string& key) const { return opts.find(key) != opts.end(); }

    //! First value given for a flag.
    //! @param key flag name, without the leading "--"
    //! @param def returned when the flag is absent
    //! @return the first value, or `def`
    std::string get(const std::string& key, const std::string& def = {}) const {
        auto it = opts.find(key);
        return it == opts.end() ? def : it->second;
    }

    //! Every value given for a flag, in command-line order.
    //! @param key flag name, without the leading "--"
    //! @return the values, empty when the flag is absent
    std::vector<std::string> getAll(const std::string& key) const {
        std::vector<std::string> v;
        auto range = opts.equal_range(key);
        for (auto it = range.first; it != range.second; ++it) v.push_back(it->second);
        return v;
    }
};

//! Parse argv.
//! @param argc,argv as received by main()
//! @return the parsed arguments
//! @note Never terminates the process -- the caller inspects
//!       @ref CliArgs::help and @ref CliArgs::valid and decides what to do.
CliArgs parseArgs(int argc, char* argv[]);

//! Pre-formatted help lines for the shared flags, so every tool describes the
//! same flag the same way. An app passes the subset it actually honours to
//! @ref printUsage; the -h/--help line is always added automatically.
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

//! Print usage for one tool.
//! @param appName name to print
//! @param desc one-line summary of the tool
//! @param options the flag lines to list, e.g. {cliopt::MJS, cliopt::LAZ};
//!        the -h/--help line is added automatically
//! @param toStderr print to stderr rather than stdout, for error reporting
void printUsage(const char* appName, const char* desc,
                const std::vector<std::string>& options, bool toStderr = false);

}  // namespace calib