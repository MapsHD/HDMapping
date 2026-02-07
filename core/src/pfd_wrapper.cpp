#include <pch/pch.h>

#include <pfd_wrapper.hpp>
#include <portable-file-dialogs.h>

namespace mandeye::fd
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
        std::vector<std::string> files;
        static std::shared_ptr<pfd::open_file> open_file;

        files = pfd::open_file(title, internal::lastLocationHint, filter, multiselect).result();

        for (auto& f : files)
        {
            f = std::filesystem::path(f).lexically_normal().string();
        }

        if (!files.empty())
        {
            std::filesystem::path pfile(files.back());
            if (pfile.has_parent_path())
            {
                internal::lastLocationHint = pfile.parent_path().string();
            }
        }
        return files;
    }

    std::string SaveFileDialog(
        const std::string& title,
        const std::vector<std::string>& filter,
        const std::string& defaultExtension,
        const std::string& defaultFileName)
    {
        std::string file;
        static std::shared_ptr<pfd::save_file> save_file;

        // build default path (directory + suggested filename)
        std::string defaultPath = internal::lastLocationHint;
        if (!defaultFileName.empty())
        {
            defaultPath = (std::filesystem::path(internal::lastLocationHint) / defaultFileName).string();
        }

        file = pfd::save_file(title, defaultPath, filter).result();

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
        std::string output_folder_name = "";

        output_folder_name = pfd::select_folder(title, internal::lastLocationHint).result();
        // std::cout << "folder: '" << output_folder_name << "'" << std::endl;

        return output_folder_name;
    }

    void OutOfMemMessage()
    {
        std::cerr << "Adjust paging / swap memory with tips available here : "
                     "https://github.com/MapsHD/HDMapping/tree/main/doc/"
                     "virtual_memory.md "
                  << std::endl;
        [[maybe_unused]] pfd::message message(
            "System is out of memory",
            "System is out memory, make sure that virtual memory is set "
            "correctly, and try again. Application will be terminated"
            "Please follow guidlines available here : "
            "https://github.com/MapsHD/HDMapping/tree/main/doc/"
            "virtual_memory.md",
            pfd::choice::ok,
            pfd::icon::error);
        message.result();
    }
} // namespace mandeye::fd