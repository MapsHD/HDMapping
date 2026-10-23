#include "staging.h"

namespace fs = std::filesystem;

namespace easy_gui
{
    bool createStagingFolder(const RecordingFiles& files, const fs::path& staging, std::string& method, std::string& error)
    {
        std::error_code ec;
        if (fs::exists(staging, ec))
        {
            error = "The work folder '" + staging.string() + "' already exists.";
            return false;
        }
        if (!fs::create_directories(staging, ec))
        {
            error = "Could not create the work folder '" + staging.string() + "' (" + ec.message() + ").";
            return false;
        }

        std::vector<fs::path> sources;
        for (size_t i = 0; i < files.pairs; i++)
        {
            sources.push_back(files.laz_files[i]);
            sources.push_back(files.csv_files[i]);
        }
        if (!files.sn_file.empty())
            sources.push_back(files.sn_file);
        if (!files.calibration_file.empty())
            sources.push_back(files.calibration_file);

        enum class Method
        {
            Symlink,
            Hardlink,
            Copy
        };
        Method m = Method::Symlink;

        for (const auto& source : sources)
        {
            const fs::path absolute_source = fs::absolute(source, ec);
            const fs::path target = staging / source.filename();
            for (;;)
            {
                ec.clear();
                if (m == Method::Symlink)
                    fs::create_symlink(absolute_source, target, ec);
                else if (m == Method::Hardlink)
                    fs::create_hard_link(absolute_source, target, ec);
                else
                    fs::copy_file(absolute_source, target, ec);
                if (!ec)
                    break;
                if (m == Method::Copy)
                {
                    error = "Could not prepare '" + source.filename().string() + "' for processing (" + ec.message() + ").";
                    return false;
                }
                // Fall back for this and all following files; anything half-created is overwritten by the next try.
                fs::remove(target, ec);
                m = (m == Method::Symlink) ? Method::Hardlink : Method::Copy;
            }
        }

        method = (m == Method::Symlink) ? "symbolic links" : (m == Method::Hardlink) ? "hard links" : "copies";
        return true;
    }
} // namespace easy_gui
