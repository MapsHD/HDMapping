#pragma once
#include <vector>
#include <string>
namespace mandeye::fd{
    namespace internal
    {
        static  std::string lastLocationHint = ".";//"C:\\";
    }
    const std::vector<std::string> LAS_LAZ_filter = {"LAS file (*.laz)", "*.laz", "LASzip file (*.las)", "*.las", "All files", "*"};
    const std::vector<std::string> ImageFilter = { "Image Files", "*.png *.jpg *.jpeg *.bmp"};
    const std::vector <std::string> LazFilter = { "LAZ files", "*.laz *.las" };
    const std::vector<std::string> Session_filter = { "Session, json", "*.json" };
    const std::vector<std::string> Resso_filter = { "Resso, reg", "*.reg" };
    const std::vector<std::string> Dxf_filter = { "dxf", "*.dxf" };
    const std::vector<std::string> Csv_filter = { "Csv", "*.csv" };
    const std::vector<std::string> All_Filter = {"All Files", "*.png *.jpg *.jpeg *.bmp *.las *.laz *.json *.dxf *.csv"};

    std::string OpenFileDialogOneFile(const std::string& title, const std::vector<std::string>&filter);
    std::vector<std::string> OpenFileDialog(const std::string& title, const std::vector<std::string>&filter, bool multiselect);
    std::string SaveFileDialog(const std::string& title, const std::vector<std::string>&filter, const std::string& defaultExtension="");
    std::string SelectFolder(const std::string& title);

    void OutOfMemMessage();
}