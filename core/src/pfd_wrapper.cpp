#include "pfd_wrapper.hpp"
#include <portable-file-dialogs.h>
#include <filesystem>
#include <cassert>

namespace mandeye::fd{
  std::string OpenFileDialogOneFile(const std::string& title, const std::vector<std::string>&filter)
  {
      auto sel = OpenFileDialog(title, filter, false);
      if (sel.empty())
      {
        return "";
      }
      return sel.back();
  }

  std::vector<std::string> OpenFileDialog(const std::string& title, const std::vector<std::string>&filter, bool multiselect)
    {
        std::vector<std::string> files;
        static std::shared_ptr<pfd::open_file> open_file;
        const auto t = [&]() {
            files = pfd::open_file(title, internal::lastLocationHint, filter, multiselect).result();

        };
        std::thread th(t);
        th.join();

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

  std::string SaveFileDialog(const std::string& title, const std::vector<std::string>& filter, const std::string& defaultExtension, const std::string& defaultFileName)
  {
      std::string file;
      static std::shared_ptr<pfd::save_file> save_file;

      // build default path (directory + suggested filename)
      std::string defaultPath = internal::lastLocationHint;
      if (!defaultFileName.empty()) {
          defaultPath = (std::filesystem::path(internal::lastLocationHint) / defaultFileName).string();
      }

      const auto t = [&]() {
          file = pfd::save_file(title, defaultPath, filter).result();
          };
      std::thread th(t);
      th.join();

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
      const auto t = [&]()
      {
        output_folder_name = pfd::select_folder(title, internal::lastLocationHint).result();
        std::cout << "folder: '" << output_folder_name << "'" << std::endl;
      };
      std::thread th(t);
      th.join();
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
            pfd::choice::ok, pfd::icon::error);
        message.result();
      }
}