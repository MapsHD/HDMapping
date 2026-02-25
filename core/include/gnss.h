#pragma once

#include <point_clouds.h>

#include <string>
#include <vector>
#include <set>
namespace CRTs
{
    // List of supported CRTs:

    constexpr const char* PUWG92_ID = "PUWG92";
    constexpr const char* UTM_AUTO_ID = "UTM_AUTO";
    constexpr const char* WEBMERC_ID = "WebMercator";
    const std::set<std::string> SupportedCRTs{ PUWG92_ID, UTM_AUTO_ID, WEBMERC_ID };
}

class GNSS
{
public:

    struct GlobalPose
    {
        double timestamp;
        double lat;
        double lon;
        double alt;
        double hdop;
        double satelites_tracked;
        double height;
        double age;
        double time;
        double fix_quality;
        double enu_x; // ENU
        double enu_y; // ENU
        double enu_z; // ENU
        double dist_xy_along;
    };

    GNSS()
    {
        ;
    };
    ~GNSS()
    {
        ;
    };

    //! \brief Get available geoid models from PROJ library
    //! \return vector of geoid model names
    static std::vector<std::string> get_available_geoids();

    //! \brief Load GNSS data from file and converts to PUWG92
    //! \param input_file_names - vector of file names
    //! \param localize - if true, the data is moved to the first point
    bool load_data_from_gnss_and_convert_to_92(const std::vector<std::string>& input_file_names, Eigen::Vector3d& out_offset, bool localize = false);
    
    //! Load data from timestamped NMEA files (timestamp + NMEA sentence) 
    //! \param input_file_names - vector of file names
    bool load_raw_data_from_nmea(const std::vector<std::string>& input_file_names);


    //! \brief Load *.GNSS files data from file without projection
    //! \param input_file_names - vector of file names
    //! \ return true if the data was loaded successfully, false otherwise
    bool load_raw_data_from_gnss(const std::vector<std::string>& input_file_names);
   
    //! Project the loaded GNSS data to Mercator projection using the WGS84 reference point
    bool project_to_mercator_projection();
    
    //! Project loaded WGS84 data to topocentric coordinates using the WGS84 reference point
    //! \see https://proj.org/en/stable/operations/conversions/topocentric.html
    bool project_using_proj(const std::string& geoidFile);

    //! Unproject the loaded data from topocentric coordinates to WGS84 using the WGS84 reference point
    std::vector<Eigen::Vector3d> unproject_using_proj(const std::vector<Eigen::Vector3d>& pointcloud, const std::string& geoidFile);

    double getGeoidSeparation(double lat_deg, double lon_deg, const std::string& geoidFile);

    //! Converts coordinate system
    //! \param llaPointcloud - pointcloud converted to LLA (WGS84)
    //! \targetCRT - target coordinate system e.g. PUWG92 or UTM
    //! \geoid - name of geoid/ellipsoid, empty for WG84 e
    std::vector<Eigen::Vector3d> CRTConvert(const std::vector<Eigen::Vector3d>& llaPointcloud, const std::string targetCRT, const std::string geoid);
    
    void render(const PointClouds& point_clouds_container);
    bool save_to_laz(const std::string& output_file_names, double offset_x, double offset_y, double offset_alt);

    std::vector<GlobalPose> gnss_poses;
    // double offset_x = 0;
    // double offset_y = 0;
    // double offset_alt = 0;
    bool show_correspondences = false;

    double WGS84ReferenceLatitude = 0;
    double WGS84ReferenceLongitude = 0;
    double geoidSeparation = 0; //! Distance from WGS84 elipsoid to geoid at reference
    bool setWGS84ReferenceFromFirstPose = true;
};
