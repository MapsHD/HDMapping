#include <CalibCore/FileDialog.h>

#include <portable-file-dialogs.h>

#include <filesystem>

namespace calib::fd
{
    std::string OpenFileDialogOneFile(const std::string& title, const std::vector<std::string>& filter)
    {
        auto sel = OpenFileDialog(title, filter, false);
        if (sel.empty())
            return "";

        return std::filesystem::path(sel.back()).lexically_normal().string();
    }

    std::vector<std::string> OpenFileDialog(const std::string& title, const std::vector<std::string>& filter, bool multiselect)
    {
        std::vector<std::string> files = pfd::open_file(title, internal::lastLocationHint, filter, multiselect).result();

        for (auto& f : files)
            f = std::filesystem::path(f).lexically_normal().string();

        if (!files.empty())
        {
            std::filesystem::path pfile(files.back());
            if (pfile.has_parent_path())
                internal::lastLocationHint = pfile.parent_path().string();
        }
        return files;
    }

    std::string SaveFileDialog(
        const std::string& title,
        const std::vector<std::string>& filter,
        const std::string& defaultExtension,
        const std::string& defaultFileName)
    {
        std::string defaultPath = internal::lastLocationHint;
        if (!defaultFileName.empty())
            defaultPath = (std::filesystem::path(internal::lastLocationHint) / defaultFileName).string();

        std::string file = pfd::save_file(title, defaultPath, filter).result();
        if (file.empty())
            return file;

        std::filesystem::path pfile(file);
        if (!pfile.has_extension())
            file += defaultExtension;

        if (pfile.has_parent_path())
            internal::lastLocationHint = pfile.parent_path().string();

        return file;
    }

    std::string SelectFolder(const std::string& title)
    {
        std::string folder = pfd::select_folder(title, internal::lastLocationHint).result();
        if (!folder.empty())
            internal::lastLocationHint = folder;
        return folder;
    }
} // namespace calib::fd
