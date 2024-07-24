#include "pfd_wrapper.hpp"
#include <portable-file-dialogs.h>
namespace mandeye::fd{
    
    std::vector<std::string> OpenFileDialog(const std::string& title, const std::vector<std::string>&filter, bool multiselect)
    {
        std::vector<std::string> files;
        static std::shared_ptr<pfd::open_file> open_file;
        const auto t = [&]() {
            files = pfd::open_file(title, "C:\\", filter, multiselect).result();

        };
        std::thread t1(t);
        t1.join();

        return files;
    }

    std::string SaveFileDialog(const std::string& title, const std::vector<std::string>&filter)
    {
        std::string file;
        static std::shared_ptr<pfd::save_file> save_file;
        const auto t = [&]() {
            file = pfd::save_file(title, "C:\\", filter).result();
        };
        std::thread t1(t);
        t1.join();
        return file;
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