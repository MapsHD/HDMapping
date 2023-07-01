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

    bool load(const std::vector<std::string> &input_file_names);
    void render(const PointClouds &point_clouds_container);

    std::vector<GlobalPose> gnss_poses;
    double offset_x = 0;
    double offset_y = 0;
    double offset_alt = 0;
    bool show_correspondences = true;
};


#endif