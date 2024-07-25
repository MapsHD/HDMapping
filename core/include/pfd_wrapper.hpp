#pragma once
#include <vector>
#include <string>
namespace mandeye::fd{
    const std::vector<std::string> ImageFilter = { "Image Files", "*.png *.jpg *.jpeg *.bmp"};
    const std::vector <std::string> LazFilter = { "LAZ files", "*.laz *.las" };
    const std::vector<std::string> Session_filter = { "Session, json", "*.json" };
    const std::vector<std::string> Resso_filter = { "Resso, reg", "*.reg" };

    std::vector<std::string> OpenFileDialog(const std::string& title, const std::vector<std::string>&filter, bool multiselect);
    std::string SaveFileDialog(const std::string& title, const std::vector<std::string>&filter);

    void OutOfMemMessage();
}