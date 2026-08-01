#pragma once

#include <string>
#include <vector>

// Native file/folder picker dialogs (portable-file-dialogs), for the
// camera_lidar_calibration app family. A small, deliberately independent
// copy of core's Core/pfd_wrapper.hpp (namespace mandeye::fd) rather than a
// reuse of it: that wrapper is only built into the GUI-enabled `core`
// target, and linking `core` here would drag in core_math/session/SLAM code
// none of these apps otherwise need. portable-file-dialogs itself is a
// single vendored header (3rdparty/portable-file-dialogs-master) with no
// dependency on `core`, so wrapping it directly is cheap.
namespace calib::fd
{
    namespace internal
    {
        static std::string lastLocationHint = ".";
    }

    const std::vector<std::string> LazFilter = { "LAS/LAZ files (*.laz, *.las)", "*.laz *.las", "All files", "*" };
    const std::vector<std::string> ImageFilter = {
        "Image files (*.bmp, *.jpg, *.jpeg, *.png)", "*.bmp *.jpg *.jpeg *.png", "All files", "*"
    };
    const std::vector<std::string> CalibJsonFilter = { "Calibration JSON (*.json)", "*.json", "All files", "*" };
    const std::vector<std::string> IntrinsicsFilter = {
        "Camera intrinsics (*.json, *.yml, *.yaml)", "*.json *.yml *.yaml", "All files", "*"
    };
    const std::vector<std::string> SessionManifestFilter = { "Mandeye session manifest (*.mjs)", "*.mjs", "All files", "*" };

    // Returns "" if the dialog was cancelled.
    std::string OpenFileDialogOneFile(const std::string& title, const std::vector<std::string>& filter);

    // Returns an empty vector if the dialog was cancelled.
    std::vector<std::string> OpenFileDialog(const std::string& title, const std::vector<std::string>& filter, bool multiselect);

    // Returns "" if the dialog was cancelled.
    std::string SaveFileDialog(
        const std::string& title,
        const std::vector<std::string>& filter,
        const std::string& defaultExtension = "",
        const std::string& defaultFileName = "");

    // Returns "" if the dialog was cancelled.
    std::string SelectFolder(const std::string& title);
} // namespace calib::fd
