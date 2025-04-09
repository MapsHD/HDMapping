#ifndef _GNSS_H_
#define _GNSS_H_

#include <string>
#include <vector>
#include <point_clouds.h>

class GNSS{
    public:

    struct GlobalPose{
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
        double x;
        double y;
    };

    GNSS(){;};
    ~GNSS(){;};

    //! \brief Load GNSS data from file and converts to PUWG92
    //! \param input_file_names - vector of file names
    //! \param localize - if true, the data is moved to the first point
    bool load(const std::vector<std::string> &input_file_names, bool localize = false);
    bool load_mercator_projection(const std::vector<std::string> &input_file_names);
    void render(const PointClouds &point_clouds_container);
    bool save_to_laz(const std::string &output_file_names, double offset_x, double offset_y, double offset_alt);

    std::vector<GlobalPose> gnss_poses;
    //double offset_x = 0;
    //double offset_y = 0;
    //double offset_alt = 0;
    bool show_correspondences = false;

    double WGS84ReferenceLatitude = 0;
    double WGS84ReferenceLongitude = 0;
    bool setWGS84ReferenceFromFirstPose = true;
};


#endif