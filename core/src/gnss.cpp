#include <gnss.h>
#include <wgs84_do_puwg92.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <WGS84toCartesian.hpp>
#include <laszip/laszip_api.h>
#include <nmea.h>
#if WITH_GUI == 1
#include <GL/freeglut.h>
#endif

inline void split(std::string &str, char delim, std::vector<std::string> &out)
{
    size_t start;
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}

bool GNSS::load(const std::vector<std::string> &input_file_names, bool localize)
{
    // 54651848940 5156.43798828125 2009.1610107421875 122.1999969482421875 1.25 8 35.799999237060546875 nan 14:51:42 1
    // timestamp lat lon alt hdop satelites_tracked height age time fix_quality

    gnss_poses.clear();

    std::cout << "loading GNSS data from following files:" << std::endl;
    for (const auto &fn : input_file_names)
    {
        std::cout << fn << std::endl;
    }

    for (const auto &fn : input_file_names)
    {
        std::ifstream infile(fn);
        if (!infile.good())
        {
            std::cout << "problem with file: '" << fn << "'" << std::endl;
            return false;
        }
        std::string s;

        while (!infile.eof())
        {
            getline(infile, s);
            std::vector<std::string> strs;
            split(s, ' ', strs);

            if (strs.size() >= 10)
            {
                GlobalPose gp;
                std::istringstream(strs[0]) >> gp.timestamp;
                std::istringstream(strs[1]) >> gp.lat;
                std::istringstream(strs[2]) >> gp.lon;
                std::istringstream(strs[3]) >> gp.alt;
                std::istringstream(strs[4]) >> gp.hdop;
                std::istringstream(strs[5]) >> gp.satelites_tracked;
                std::istringstream(strs[6]) >> gp.height;
                std::istringstream(strs[7]) >> gp.age;
                std::istringstream(strs[8]) >> gp.time;
                std::istringstream(strs[9]) >> gp.fix_quality;

                double L = gp.lon;
                double B = gp.lat;

                wgs84_do_puwg92(B, L, &gp.y, &gp.x);

                if (Eigen::Vector3d(gp.y, gp.x, gp.alt).norm() > 0)
                {
                    gnss_poses.push_back(gp);
                }
            }
        }

        infile.close();
    }

    std::sort(gnss_poses.begin(), gnss_poses.end(), [](GNSS::GlobalPose &a, GNSS::GlobalPose &b)
              { return (a.timestamp < b.timestamp); });

    auto firstGNSSIt = std::find_if(gnss_poses.begin(), gnss_poses.end(), [](const GNSS::GlobalPose &gp)
                                    { return gp.x != 0 && gp.y != 0 && gp.alt != 0; });
    if (firstGNSSIt == gnss_poses.end())
    {
        std::cout << "no valid GNSS data" << std::endl;
        return false;
    }

    auto firstGNSS = *firstGNSSIt;
    std::cout << std::setprecision(20);

    std::cout << std::setprecision(20);
    std::cout << "firstGNSS: " << firstGNSS.lat << " " << firstGNSS.lon << " " << firstGNSS.alt << std::endl;
    std::cout << "firstGNSS: " << firstGNSS.x << " " << firstGNSS.y << " " << firstGNSS.alt << std::endl;

    WGS84ReferenceLatitude = firstGNSS.lat;
    WGS84ReferenceLongitude = firstGNSS.lon;

    if (localize)
    {
        for (auto &pose : gnss_poses)
        {
            pose.x = pose.x - firstGNSS.x;
            pose.y = pose.y - firstGNSS.y;
            pose.alt = pose.alt - firstGNSS.alt;
            std::cout << "pose.x: " << pose.x << " pose.y: " << pose.y << " pose.alt: " << pose.alt << std::endl;
        }
    }

    std::cout << std::setprecision(20);
    std::cout << "firstGNSS: " << firstGNSS.lat << " " << firstGNSS.lon << " " << firstGNSS.alt << std::endl;
    std::cout << "firstGNSS: " << firstGNSS.x << " " << firstGNSS.y << " " << firstGNSS.alt << std::endl;
    
    return true;
}

bool GNSS::load_mercator_projection(const std::vector<std::string> &input_file_names)
{
    gnss_poses.clear();

    std::cout << "loading GNSS data from following files:" << std::endl;
    for (const auto &fn : input_file_names)
    {
        std::cout << fn << std::endl;
    }

    for (const auto &fn : input_file_names)
    {
        std::ifstream infile(fn);
        if (!infile.good())
        {
            std::cout << "problem with file: '" << fn << "'" << std::endl;
            return false;
        }
        std::string s;
        while (!infile.eof())
        {
            getline(infile, s);
            std::vector<std::string> strs;
            split(s, ' ', strs);

            if (strs.size() >= 10)
            {
                GlobalPose gp;
                std::istringstream(strs[0]) >> gp.timestamp;
                std::istringstream(strs[1]) >> gp.lat;
                std::istringstream(strs[2]) >> gp.lon;
                std::istringstream(strs[3]) >> gp.alt;
                std::istringstream(strs[4]) >> gp.hdop;
                std::istringstream(strs[5]) >> gp.satelites_tracked;
                std::istringstream(strs[6]) >> gp.height;
                std::istringstream(strs[7]) >> gp.age;
                std::istringstream(strs[8]) >> gp.time;
                std::istringstream(strs[9]) >> gp.fix_quality;

                if (gp.lat == gp.lat)
                {
                    if (gp.lon == gp.lon)
                    {
                        if (gp.alt == gp.alt)
                        {
                            if (gp.lat != 0)
                            {
                                if (gp.lon != 0)
                                {
                                    gnss_poses.push_back(gp);
                                   // std::cout << std::setprecision(20);
                                  //  std::cout << "gp.lat " << gp.lat << " gp.lon " << gp.lon << " gp.alt " << gp.alt << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
        infile.close();
    }

    std::sort(gnss_poses.begin(), gnss_poses.end(), [](GNSS::GlobalPose &a, GNSS::GlobalPose &b)
              { return (a.timestamp < b.timestamp); });

    std::array<double, 2> WGS84Reference{0, 0};

    if (gnss_poses.size() > 0)
    {
        if (setWGS84ReferenceFromFirstPose)
        {
            WGS84Reference[0] = gnss_poses[0].lat;
            WGS84Reference[1] = gnss_poses[0].lon;
            WGS84ReferenceLatitude = gnss_poses[0].lat;
            WGS84ReferenceLongitude = gnss_poses[0].lon;
        }
        else
        {
            WGS84Reference[0] = WGS84ReferenceLatitude;
            WGS84Reference[1] = WGS84ReferenceLongitude;
        }
    }

    for (int i = 0; i < gnss_poses.size(); i++)
    {
        std::array<double, 2> WGS84Position{gnss_poses[i].lat, gnss_poses[i].lon};
        std::array<double, 2> result{wgs84::toCartesian(WGS84Reference, WGS84Position)};
        gnss_poses[i].x = result[0];
        gnss_poses[i].y = result[1];
    }

    return true;
}

double dm_to_dd(const std::string &dm, char direction, bool is_latitude)
{
    if (dm.empty())
        return 0.0;

    size_t deg_len = is_latitude ? 2 : 3;

    double degrees = std::stod(dm.substr(0, deg_len));
    double minutes = std::stod(dm.substr(deg_len));

    double dd = degrees + minutes / 60.0;
    if (direction == 'S' || direction == 'W')
        dd *= -1.0;

    return dd;
}


bool GNSS::load_nmea_mercator_projection(const std::vector<std::string> &input_file_names)
{
    gnss_poses.clear();

    std::cout << "loading NMEA data from following files:" << std::endl;
    for (const auto &fn : input_file_names)
    {
        std::cout << fn << std::endl;
    }
    std::optional<hd_mapping::nmea::GNGGAData> lastGGAData;
    std::optional<hd_mapping::nmea::GNRMCData> lastRMCData;
    for (const auto &fn : input_file_names)
    {
        std::ifstream infile(fn);
        if (!infile.good())
        {
            std::cout << "problem with file: '" << fn << "'" << std::endl;
            return false;
        }
        std::string line;
        while (!infile.eof())
        {
            getline(infile, line);
            using namespace hd_mapping::nmea;

            auto [timestampLidar, timestampUnix, nmeaSentence] = BreakLineFromNMEAFile(line);
            if (!validateNMEAChecksum(nmeaSentence)) {
                  std::cout << "Invalid NMEA checksum in line: " << line << std::endl;
                  continue;
            }

            const auto gga = parseGNGGA(nmeaSentence);
            const auto rmc = parseGNRMC(nmeaSentence);
            if (rmc.has_value()) {
                lastRMCData = rmc;
            }
            if (gga.has_value()) {
                    lastGGAData = gga;
            }
            if (gga.has_value()) {
              GlobalPose gp;
              gp.timestamp = timestampLidar;
              gp.lat = gga->latitude;
              gp.lon = gga->longitude;
              gp.alt = gga->altitude;
              gp.hdop = gga->hdop;
              gp.height = gga->altitude;
              gp.age = gga->age_of_data;
              gp.time = 0; // todo convert to seconds from string
              gp.fix_quality = gga->fix_quality;

              // register if there  is not nans
              if (gp.lat == gp.lat && gp.lon == gp.lon &&
                  gp.alt == gp.alt) {
                gnss_poses.push_back(gp);
              }
            }
        }
        infile.close();
    }

    std::cout << "loaded " << gnss_poses.size() << " gps poses" << std::endl;
    
    std::sort(gnss_poses.begin(), gnss_poses.end(), [](GNSS::GlobalPose &a, GNSS::GlobalPose &b)
                                                  { return (a.timestamp < b.timestamp); });

    std::array<double, 2> WGS84Reference{0, 0};

    if (gnss_poses.size() > 0)
    {
        if (setWGS84ReferenceFromFirstPose)
        {
            WGS84Reference[0] = gnss_poses[0].lat;
            WGS84Reference[1] = gnss_poses[0].lon;
            WGS84ReferenceLatitude = gnss_poses[0].lat;
            WGS84ReferenceLongitude = gnss_poses[0].lon;
        }
        else
        {
            WGS84Reference[0] = WGS84ReferenceLatitude;
            WGS84Reference[1] = WGS84ReferenceLongitude;
        }
    }

    for (int i = 0; i < gnss_poses.size(); i++)
    {
        std::array<double, 2> WGS84Position{gnss_poses[i].lat, gnss_poses[i].lon};
        std::array<double, 2> result{wgs84::toCartesian(WGS84Reference, WGS84Position)};
        gnss_poses[i].x = result[0];
        gnss_poses[i].y = result[1];
    }

    return true;
}

#if WITH_GUI == 1
void GNSS::render(const PointClouds &point_clouds_container)
{
    glColor3f(1, 1, 1);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < gnss_poses.size(); i++)
    {
        glVertex3f(gnss_poses[i].x - point_clouds_container.offset.x(), gnss_poses[i].y - point_clouds_container.offset.y(), gnss_poses[i].alt - point_clouds_container.offset.z());
    }
    glEnd();

    if (show_correspondences)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_LINES);
        for (const auto &pc : point_clouds_container.point_clouds)
        {
            for (int i = 0; i < gnss_poses.size(); i++)
            {
                double time_stamp = gnss_poses[i].timestamp;

                auto it = std::lower_bound(pc.local_trajectory.begin(), pc.local_trajectory.end(),
                                           time_stamp, [](const PointCloud::LocalTrajectoryNode &lhs, const double &time) -> bool
                                           { return lhs.timestamps.first < time; });

                int index = it - pc.local_trajectory.begin();

                if (index > 0 && index < pc.local_trajectory.size())
                {

                    if (fabs(time_stamp - pc.local_trajectory[index].timestamps.first) < 10e12)
                    {
                        auto m = pc.m_pose * pc.local_trajectory[index].m_pose;
                        glVertex3f(m(0, 3), m(1, 3), m(2, 3));

                        glVertex3f(gnss_poses[i].x - point_clouds_container.offset.x(), gnss_poses[i].y - point_clouds_container.offset.y(), gnss_poses[i].alt - point_clouds_container.offset.z());
                    }
                }
            }
        }
        glEnd();
    }
}
#endif

bool GNSS::save_to_laz(const std::string &output_file_names, double offset_x, double offset_y, double offset_alt)
{
    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    Eigen::Vector3d max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    Eigen::Vector3d min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

    for (int i = 0; i < gnss_poses.size(); i++)
    {
        max.x() = std::max(max.x(), gnss_poses[i].x);
        max.y() = std::max(max.y(), gnss_poses[i].y);
        max.z() = std::max(max.z(), gnss_poses[i].alt);

        min.x() = std::min(min.x(), gnss_poses[i].x);
        min.y() = std::min(min.y(), gnss_poses[i].y);
        min.z() = std::min(min.z(), gnss_poses[i].alt);
    }

    // create the writer
    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: creating laszip writer\n");
        return false;
    }

    // get a pointer to the header of the writer so we can populate it

    laszip_header *header;

    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
        return false;
    }

    // populate the header

    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0); // see LAS specification for details
    header->version_major = 1;
    header->version_minor = 2;
    //    header->file_creation_day = 120;
    //    header->file_creation_year = 2013;
    header->point_data_format = 1;
    header->point_data_record_length = 0;
    header->number_of_point_records = gnss_poses.size();
    header->number_of_points_by_return[0] = gnss_poses.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length = 28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = max.x();
    header->min_x = min.x();
    header->max_y = max.y();
    header->min_y = min.y();
    header->max_z = max.z();
    header->min_z = min.z();

    header->x_offset = offset_x;
    header->y_offset = offset_y;
    header->z_offset = offset_alt;

    // optional: use the bounding box and the scale factor to create a "good" offset
    // open the writer
    laszip_BOOL compress = (strstr(output_file_names.c_str(), ".laz") != 0);

    if (laszip_open_writer(laszip_writer, output_file_names.c_str(), compress))
    {
        fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", output_file_names.c_str());
        return false;
    }

    fprintf(stderr, "writing file '%s' %scompressed\n", output_file_names.c_str(), (compress ? "" : "un"));

    // get a pointer to the point of the writer that we will populate and write

    laszip_point *point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i = 0; i < gnss_poses.size(); i++)
    {
        point->intensity = 0;

        const auto &p = gnss_poses[i];
        p_count++;
        coordinates[0] = p.x;
        coordinates[1] = p.y;
        coordinates[2] = p.alt;
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }

        if (laszip_write_point(laszip_writer))
        {
            fprintf(stderr, "DLL ERROR: writing point %I64d\n", p_count);
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr, "DLL ERROR: getting point count\n");
        return false;
    }

    fprintf(stderr, "successfully written %I64d points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: closing laszip writer\n");
        return false;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
        return false;
    }

    std::cout << "exportLaz DONE" << std::endl;
    return true;
}