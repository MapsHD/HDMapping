#pragma once

#include "preflight.h"

#include <filesystem>
#include <string>

namespace easy_gui
{
    //! Fills `staging` (created, must not exist yet) with links to the files step 1 will read: the paired point cloud
    //! and IMU files, the .sn file and the calibration file.
    //!
    //! Why: load_data() writes its cache/ and preview/ folders next to its input files, and the recording may be on
    //! the scanner's USB stick. Step 1 runs on the staging folder instead, so nothing is ever written into the
    //! recording. Only the paired files are linked, which is exactly what load_data would load anyway.
    //!
    //! Tries a symbolic link per file, then a hard link, then a copy (Windows without developer mode cannot create
    //! symlinks; hard links only work on the same volume). `method` reports which one was used.
    bool createStagingFolder(const RecordingFiles& files, const std::filesystem::path& staging, std::string& method, std::string& error);
} // namespace easy_gui
