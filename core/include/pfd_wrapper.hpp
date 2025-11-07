#pragma once
#include <vector>
#include <string>
namespace mandeye::fd{
    namespace internal
    {
        static  std::string lastLocationHint = ".";//"C:\\";
    }
    const std::vector<std::string> LAS_LAZ_filter = {"LASzip file (*.laz)", "*.laz", "LAS file (*.las)", "*.las", "All files", "*"};
    const std::vector <std::string> LazFilter = { "LAZ files (*.laz)", "*.laz *.las" };
    const std::vector<std::string> ImageFilter = { "Image files", "*.png *.jpg *.jpeg *.bmp"};

    const std::vector<std::string> Calibration_filter = { "Mandeye JSON Calibration (*.mjc)", "*.mjc", "Generic JSON (*.json)", "*.json" };
    const std::vector<std::string> Session_filter = { "Mandeye JSON Session (*.mjs)", "*.mjs", "Generic JSON (*.json)", "*.json" };
    const std::vector<std::string> Project_filter = { "Mandeye JSON Project (*.mjp)", "*.mjp", "Generic JSON (*.json)", "*.json" };

    const std::vector<std::string> IniPoses_filter = { "Mandeye REG Initial poses (*.mri)", "*.mri", "Generic REG initial poses (*.reg)", "*.reg" };
    const std::vector<std::string> Poses_filter = { "Mandeye REG Poses (*.mrp)", "*.mrp", "Generic REG poses (*.reg)", "*.reg" };

    const std::vector<std::string> Resso_filter = { "Resso (*.reg)", "*.reg" };
    const std::vector<std::string> Dxf_filter = { "dxf (*.dxf)", "*.dxf" };
    const std::vector<std::string> Csv_filter = { "Csv (*.csv)", "*.csv" };
    const std::vector<std::string> Toml_filter = {"Toml (*.toml)", "*.toml"};
    const std::vector<std::string> All_Filter = {"All files", "*.png *.jpg *.jpeg *.bmp *.las *.laz *.json *.dxf *.csv *.sn"};
    const std::vector<std::string> json_filter = { "Calibration file (*.json)", "*.json", "All files", "*" };
    const std::vector<std::string> sn_filter = { "sn file (*.sn)", "*.sn", "All files", "*" };

    std::string OpenFileDialogOneFile(const std::string& title, const std::vector<std::string>&filter);
    std::vector<std::string> OpenFileDialog(const std::string& title, const std::vector<std::string>&filter, bool multiselect);
    std::string SaveFileDialog(const std::string& title, const std::vector<std::string>& filter, const std::string& defaultExtension = "", const std::string& defaultFileName = "");
    std::string SelectFolder(const std::string& title);

    void OutOfMemMessage();
}