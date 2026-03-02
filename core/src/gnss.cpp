#include <pch/pch.h>

#include <WGS84toCartesian.hpp>
#include <gnss.h>
#include <nmea.h>
#include <wgs84_do_puwg92.h>

#if WITH_GUI == 1
#include <GL/freeglut.h>
#endif

#include <cstdlib>
#include <laszip/laszip_api.h>
#include <proj.h>
const std::string Elipsooid = "WGS84Ellipsoid";

// path to find
const std::vector<std::string> PathCandidate{
    std::filesystem::current_path().string(),
    std::filesystem::current_path().string() + "/proj",
    std::filesystem::current_path().string() + "/share/proj",
    std::filesystem::current_path().string() + "/data/proj",
    "/usr/local/share/proj",
    "/usr/share/proj",
    "C:/HDMapping",
};

std::vector<std::string> GNSS::get_available_geoids()
{
    std::set<std::string> unique;
    unique.insert(Elipsooid);

    // Respect HD_MAPPING_PROJ environment variable if set. It can point to a directory
    // containing geoid grid files (*.gtx). If present, search it first.
    std::vector<std::string> paths = PathCandidate;
    const char* env_proj = std::getenv("HD_MAPPING_ASSETS");
    if (env_proj && std::string(env_proj).size() > 0)
    {
        // insert at front so user-specified path is searched first
        paths.insert(paths.begin(), std::string(env_proj));
        std::cout << "HD_MAPPING_ASSETS set: " << env_proj << std::endl;
    }

    for (const auto& path : paths)
    {
        std::filesystem::path proj_path(path);
        std::error_code ec;
        if (std::filesystem::exists(proj_path, ec) && !ec)
        {
            std::filesystem::directory_iterator dir_it(proj_path, ec);
            if (ec)
            {
                std::cerr << "cannot open directory " << proj_path << ": " << ec.message() << std::endl;
                continue;
            }
            for (; dir_it != std::filesystem::directory_iterator(); dir_it.increment(ec))
            {
                if (ec)
                {
                    std::cerr << "error iterating directory " << proj_path << ": " << ec.message() << std::endl;
                    break;
                }
                const auto& entry = *dir_it;
                if (entry.is_regular_file(ec) && !ec && entry.path().extension() == ".gtx")
                    if (entry.is_regular_file() && entry.path().extension() == ".gtx")
                    {
                        std::cout << "found geoid model: " << entry.path().string() << std::endl;
                        unique.insert(entry.path().string());
                    }
            }
        }
    }

    return std::vector<std::string>(unique.begin(), unique.end());
}

#include <spdlog/spdlog.h>

inline void split(std::string& str, char delim, std::vector<std::string>& out)
{
    size_t start;
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}

bool GNSS::load_data_from_gnss_and_convert_to_92(
    const std::vector<std::string>& input_file_names, Eigen::Vector3d& out_offset, bool localize)
{
    // 54651848940 5156.43798828125 2009.1610107421875 122.1999969482421875 1.25 8 35.799999237060546875 nan 14:51:42 1
    // timestamp lat lon alt hdop satelites_tracked height age time fix_quality

    out_offset.x() = 0;
    out_offset.y() = 0;
    out_offset.z() = 0;

    gnss_poses.clear();

    std::cout << "loading GNSS data from following files:" << std::endl;
    for (const auto& fn : input_file_names)
    {
        std::cout << fn << std::endl;
    }

    for (const auto& fn : input_file_names)
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

                wgs84_do_puwg92(B, L, &gp.enu_y, &gp.enu_x);
                gp.enu_z = gp.alt;
                if (Eigen::Vector3d(gp.enu_y, gp.enu_x, gp.alt).norm() > 0)
                {
                    gnss_poses.push_back(gp);
                }
            }
        }

        infile.close();
    }

    std::sort(
        gnss_poses.begin(),
        gnss_poses.end(),
        [](GNSS::GlobalPose& a, GNSS::GlobalPose& b)
        {
            return (a.timestamp < b.timestamp);
        });

    auto firstGNSSIt = std::find_if(
        gnss_poses.begin(),
        gnss_poses.end(),
        [](const GNSS::GlobalPose& gp)
        {
            return gp.enu_x != 0 && gp.enu_y != 0 && gp.enu_z != 0;
        });
    if (firstGNSSIt == gnss_poses.end())
    {
        std::cout << "no valid GNSS data" << std::endl;
        return false;
    }

    auto firstGNSS = *firstGNSSIt;
    std::cout << std::setprecision(20);

    std::cout << std::setprecision(20);
    std::cout << "firstGNSS: " << firstGNSS.lat << " " << firstGNSS.lon << " " << firstGNSS.alt << std::endl;
    std::cout << "firstGNSS: " << firstGNSS.enu_x << " " << firstGNSS.enu_y << " " << firstGNSS.enu_z << std::endl;

    WGS84ReferenceLatitude = firstGNSS.lat;
    WGS84ReferenceLongitude = firstGNSS.lon;

    out_offset.x() = firstGNSS.enu_x;
    out_offset.y() = firstGNSS.enu_y;
    out_offset.z() = firstGNSS.enu_z;

    if (localize)
    {
        for (auto& pose : gnss_poses)
        {
            pose.enu_x = pose.enu_x - firstGNSS.enu_x;
            pose.enu_y = pose.enu_y - firstGNSS.enu_y;
            pose.enu_z = pose.enu_z - firstGNSS.enu_z;
            // std::cout << "pose.x: " << pose.x << " pose.y: " << pose.y << " pose.alt: " << pose.alt << std::endl;
        }
    }

    std::cout << std::setprecision(20);
    std::cout << "firstGNSS: " << firstGNSS.lat << " " << firstGNSS.lon << " " << firstGNSS.alt << std::endl;
    std::cout << "firstGNSS: " << firstGNSS.enu_x << " " << firstGNSS.enu_y << " " << firstGNSS.enu_z << std::endl;

    return true;
}

bool GNSS::load_raw_data_from_gnss(const std::vector<std::string>& input_file_names)
{
    gnss_poses.clear();

    std::cout << "loading GNSS data from following files:" << std::endl;
    for (const auto& fn : input_file_names)
    {
        std::cout << fn << std::endl;
    }

    for (const auto& fn : input_file_names)
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
                if (std::isfinite(gp.lat) && std::isfinite(gp.lon) && std::isfinite(gp.alt) && gp.lat != 0.0 && gp.lon != 0.0)
                {
                    gnss_poses.push_back(gp);
                }
            }
        }
        infile.close();
    }

    std::sort(
        gnss_poses.begin(),
        gnss_poses.end(),
        [](GNSS::GlobalPose& a, GNSS::GlobalPose& b)
        {
            return (a.timestamp < b.timestamp);
        });
    return true;
}

bool GNSS::project_to_mercator_projection()
{
    std::array<double, 2> WGS84Reference{ 0, 0 };

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
        std::array<double, 2> WGS84Position{ gnss_poses[i].lat, gnss_poses[i].lon };
        std::array<double, 2> result{ wgs84::toCartesian(WGS84Reference, WGS84Position) };
        gnss_poses[i].enu_x = result[0];
        gnss_poses[i].enu_y = result[1];
        gnss_poses[i].enu_z = gnss_poses[i].alt;
    }

    return true;
}

double get_ellipsoid_height(const GNSS::GlobalPose& pose)
{
    return pose.alt + pose.height;
}

bool GNSS::project_using_proj()
{
    if (gnss_poses.empty())
        return false;

    // -------------------------------------------------
    // Determine reference
    // -------------------------------------------------
    double refLat = 0.0;
    double refLon = 0.0;
    double refAlt = 0.0; // assume 0 if you don't store altitude

    if (setWGS84ReferenceFromFirstPose)
    {
        refLat = gnss_poses.front().lat;
        refLon = gnss_poses.front().lon;
        refAlt = get_ellipsoid_height(gnss_poses.front());

        WGS84ReferenceLatitude = refLat;
        WGS84ReferenceLongitude = refLon;
    }
    else
    {
        refLat = WGS84ReferenceLatitude;
        refLon = WGS84ReferenceLongitude;
        refAlt = 0.0;
    }

    // -------------------------------------------------
    // Create PROJ context + pipeline
    // -------------------------------------------------
    PJ_CONTEXT* ctx = proj_context_create();

    std::string pipeline = "+proj=pipeline "
                           "+step +proj=cart +ellps=WGS84 "
                           "+step +proj=topocentric "
                           "+ellps=WGS84 "
                           "+lat_0=" +
        std::to_string(refLat) + " +lon_0=" + std::to_string(refLon) + " +h_0=" + std::to_string(refAlt);

    PJ* P = proj_create(ctx, pipeline.c_str());
    if (!P)
    {
        proj_context_destroy(ctx);
        return false;
    }

    // -------------------------------------------------
    // Transform all GNSS poses
    // -------------------------------------------------
    for (auto& pose : gnss_poses)
    {
        PJ_COORD geo = proj_coord(
            pose.lon * M_PI / 180.0, // longitude in radians
            pose.lat * M_PI / 180.0, // latitude in radians
            get_ellipsoid_height(pose),
            0);

        PJ_COORD enu = proj_trans(P, PJ_FWD, geo);

        pose.enu_x = enu.xyz.x; // East (meters)
        pose.enu_y = enu.xyz.y; // North (meters)
        pose.enu_z = enu.xyz.z; //
    }

    proj_destroy(P);
    proj_context_destroy(ctx);

    return true;
}

double dm_to_dd(const std::string& dm, char direction, bool is_latitude)
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

std::vector<Eigen::Vector3d> GNSS::unproject_using_proj(const std::vector<Eigen::Vector3d>& pointcloud)
{
    PJ_CONTEXT* ctx = proj_context_create();

    const double alt = get_ellipsoid_height(gnss_poses.front());

    std::string pipeline = "+proj=pipeline "
                           "+step +inv +proj=topocentric "
                           "+lat_0=" +
        std::to_string(WGS84ReferenceLatitude) +
        " "
        "+lon_0=" +
        std::to_string(WGS84ReferenceLongitude) +
        " "
        "+h_0=" +
        std::to_string(alt) +
        " "
        "+ellps=WGS84 "
        "+step +inv +proj=cart +ellps=WGS84";
    PJ* P = proj_create(ctx, pipeline.c_str());
    if (!P)
    {
        proj_context_destroy(ctx);
        std::cerr << "Failed to create projection pipeline " << pipeline << std::endl;
        return {};
    }

    std::vector<Eigen::Vector3d> lla_points;
    lla_points.reserve(pointcloud.size());

    for (const auto& p : pointcloud)
    {
        PJ_COORD c;
        c.xyz.x = p.x(); // East
        c.xyz.y = p.y(); // North
        c.xyz.z = p.z(); // Up

        PJ_COORD r = proj_trans(P, PJ_FWD, c);

        double lon = r.lpzt.lam * 180.0 / M_PI;
        double lat = r.lpzt.phi * 180.0 / M_PI;
        double h_msl = r.lpzt.z;

        lla_points.emplace_back(lat, lon, h_msl);
    }

    proj_destroy(P);
    proj_context_destroy(ctx);

    return lla_points;
}

bool GNSS::load_raw_data_from_nmea(const std::vector<std::string>& input_file_names)
{
    gnss_poses.clear();

    std::cout << "loading NMEA data from following files:" << std::endl;
    for (const auto& fn : input_file_names)
    {
        std::cout << fn << std::endl;
    }
    std::optional<hd_mapping::nmea::GNGGAData> lastGGAData;
    std::optional<hd_mapping::nmea::GNRMCData> lastRMCData;
    for (const auto& fn : input_file_names)
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
            if (!validateNMEAChecksum(nmeaSentence))
            {
                std::cout << "Invalid NMEA checksum in line: " << line << std::endl;
                continue;
            }

            const auto gga = parseGNGGA(nmeaSentence);
            const auto rmc = parseGNRMC(nmeaSentence);
            if (rmc.has_value())
            {
                lastRMCData = rmc;
            }
            if (gga.has_value())
            {
                lastGGAData = gga;
            }
            if (gga.has_value())
            {
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
                if (gp.lat == gp.lat && gp.lon == gp.lon && gp.alt == gp.alt)
                {
                    gnss_poses.push_back(gp);
                }
            }
        }
        infile.close();
    }

    std::cout << "loaded " << gnss_poses.size() << " gps poses" << std::endl;

    std::sort(
        gnss_poses.begin(),
        gnss_poses.end(),
        [](GNSS::GlobalPose& a, GNSS::GlobalPose& b)
        {
            return (a.timestamp < b.timestamp);
        });

    return true;
}

std::vector<Eigen::Vector3d> GNSS::CRTConvert(
    const std::vector<Eigen::Vector3d>& llaPointcloud, const std::string targetCRT, const std::string geoid)
{
    std::vector<Eigen::Vector3d> result;
    result.reserve(llaPointcloud.size());

    if (llaPointcloud.empty())
        return result;
    if (!CRTs::SupportedCRTs.contains(targetCRT))
    {
        std::cerr << "Unsuported CRT " << targetCRT << std::endl;
        return result;
    }

    std::string target = "";

    if (targetCRT == CRTs::PUWG92_ID)
    {
        target = "EPSG:2180";
    }
    else if (targetCRT == CRTs::WEBMERC_ID)
    {
        target = "EPSG:3857";
    }
    else if (targetCRT == CRTs::UTM_AUTO_ID)
    {
        double lat = llaPointcloud[0].x();
        double lon = llaPointcloud[0].y();

        int zone = int(std::floor((lon + 180.0) / 6.0)) + 1;
        bool north = (lat >= 0.0);

        target = "+proj=utm +zone=" + std::to_string(zone) + (north ? " +north " : " +south ") + "+datum=WGS84 +units=m +no_defs";
    }

    std::cout << "target CRT " << target << std::endl;

    PJ_CONTEXT* ctx = proj_context_create();
    PJ* P_geoid = nullptr;

    if (!(geoid.empty() || geoid == Elipsooid))
    {
        std::string pipeline = "+proj=pipeline "
                               "+step +proj=unitconvert +xy_in=deg +xy_out=rad "
                               "+step +proj=vgridshift +grids=" +
            geoid +
            " "
            "+step +proj=unitconvert +xy_in=rad +xy_out=deg";

        P_geoid = proj_create(ctx, pipeline.c_str());

        if (!P_geoid)
        {
            std::cerr << "Failed to create geoid transformation " << pipeline << "\n";
            proj_context_destroy(ctx);
            return result;
        }
    }

    // ---------------------------
    // CRS transform (LLA to target)
    // ---------------------------
    PJ* P_crs = proj_create_crs_to_crs(
        ctx,
        "EPSG:4979", // WGS84 3D
        target.c_str(),
        nullptr);

    if (!P_crs)
    {
        std::cerr << "Failed to create CRS transformation\n";
        if (P_geoid)
            proj_destroy(P_geoid);
        proj_context_destroy(ctx);
        return result;
    }

    P_crs = proj_normalize_for_visualization(ctx, P_crs);
    if (!P_crs)
    {
        std::cerr << "Failed to normalize CRS transformation for visualization\n";
        proj_destroy(P_crs);
        return {};
    }

    // ---------------------------
    // Transform loop
    // ---------------------------
    for (const auto& p : llaPointcloud)
    {
        double lat = p.x();
        double lon = p.y();
        double h = p.z();

        // Geoid correction (if requested)
        if (P_geoid)
        {
            PJ_COORD c_geo = proj_coord(lon, lat, h, 0);
            PJ_COORD r_geo = proj_trans(P_geoid, PJ_FWD, c_geo);

            lon = r_geo.lpzt.lam;
            lat = r_geo.lpzt.phi;
            h = r_geo.lpzt.z;
        }

        // CRS conversion
        PJ_COORD c = proj_coord(lon, lat, h, 0);
        PJ_COORD r = proj_trans(P_crs, PJ_FWD, c);

        result.emplace_back(
            r.xy.x, // Easting / X
            r.xy.y, // Northing / Y
            r.xyz.z // height (after geoid if applied)
        );
    }

    // ---------------------------
    // Cleanup
    // ---------------------------
    proj_destroy(P_crs);
    if (P_geoid)
        proj_destroy(P_geoid);
    proj_context_destroy(ctx);

    return result;
}

double GNSS::getGeoidSeparation(double lat_deg, double lon_deg, const std::string& geoidFile)
{
    PJ_CONTEXT* ctx = proj_context_create();

    std::string pipeline = "+proj=pipeline "
                           "+step +proj=unitconvert +xy_in=deg +xy_out=rad "
                           "+step +proj=vgridshift +grids=" +
        geoidFile +
        " "
        "+step +proj=unitconvert +xy_in=rad +xy_out=deg";

    PJ* P = proj_create(ctx, pipeline.c_str());
    if (!P)
    {
        proj_context_destroy(ctx);
        return 0.0;
    }

    PJ_COORD c = proj_coord(lon_deg, lat_deg, 0.0, 0);
    PJ_COORD r = proj_trans(P, PJ_FWD, c);

    proj_destroy(P);
    proj_context_destroy(ctx);

    return -r.lpzt.z;
}

#if WITH_GUI == 1
void GNSS::render(const PointClouds& point_clouds_container)
{
    glColor3f(1, 1, 1);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < gnss_poses.size(); i++)
    {
        glVertex3f(
            gnss_poses[i].enu_x - point_clouds_container.offset.x(),
            gnss_poses[i].enu_y - point_clouds_container.offset.y(),
            gnss_poses[i].enu_z - point_clouds_container.offset.z());
    }
    glEnd();

    if (show_correspondences)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_LINES);
        for (const auto& pc : point_clouds_container.point_clouds)
        {
            for (int i = 0; i < gnss_poses.size(); i++)
            {
                double time_stamp = gnss_poses[i].timestamp;

                auto it = std::lower_bound(
                    pc.local_trajectory.begin(),
                    pc.local_trajectory.end(),
                    time_stamp,
                    [](const PointCloud::LocalTrajectoryNode& lhs, const double& time) -> bool
                    {
                        return lhs.timestamps.first < time;
                    });

                int index = it - pc.local_trajectory.begin();

                if (index > 0 && index < pc.local_trajectory.size())
                {
                    if (fabs(time_stamp - pc.local_trajectory[index].timestamps.first) < 10e12)
                    {
                        auto m = pc.m_pose * pc.local_trajectory[index].m_pose;
                        glVertex3f(m(0, 3), m(1, 3), m(2, 3));

                        glVertex3f(
                            gnss_poses[i].enu_x - point_clouds_container.offset.x(),
                            gnss_poses[i].enu_y - point_clouds_container.offset.y(),
                            gnss_poses[i].enu_z - point_clouds_container.offset.z());
                    }
                }
            }
        }
        glEnd();
    }
}
#endif

bool GNSS::save_to_laz(const std::string& output_file_names, double offset_x, double offset_y, double offset_alt)
{
    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    Eigen::Vector3d max(
        std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    Eigen::Vector3d min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

    for (int i = 0; i < gnss_poses.size(); i++)
    {
        max.x() = std::max(max.x(), gnss_poses[i].enu_x);
        max.y() = std::max(max.y(), gnss_poses[i].enu_y);
        max.z() = std::max(max.z(), gnss_poses[i].enu_z);

        min.x() = std::min(min.x(), gnss_poses[i].enu_x);
        min.y() = std::min(min.y(), gnss_poses[i].enu_y);
        min.z() = std::min(min.z(), gnss_poses[i].enu_z);
    }

    // create the writer
    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        spdlog::error("DLL ERROR: creating laszip writer");
        return false;
    }

    laszip_header* header;

    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        spdlog::error("DLL ERROR: getting header pointer from laszip writer");
        return false;
    }

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

    laszip_BOOL compress = (strstr(output_file_names.c_str(), ".laz") != 0);

    if (laszip_open_writer(laszip_writer, output_file_names.c_str(), compress))
    {
        spdlog::error("DLL ERROR: opening laszip writer for '{}'", output_file_names);
        return false;
    }

    spdlog::info("writing file '{}' {}compressed\n", output_file_names, (compress ? "" : "un"));

    laszip_point* point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        spdlog::error("DLL ERROR: getting point pointer from laszip writer");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i = 0; i < gnss_poses.size(); i++)
    {
        point->intensity = 0;

        const auto& p = gnss_poses[i];
        p_count++;
        coordinates[0] = p.enu_x;
        coordinates[1] = p.enu_y;
        coordinates[2] = p.enu_z;
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            spdlog::error("DLL ERROR: setting coordinates for point {}", p_count);
            return false;
        }

        if (laszip_write_point(laszip_writer))
        {
            spdlog::error("DLL ERROR: writing point {}", p_count);
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        spdlog::error("DLL ERROR: getting point count");
        return false;
    }

    spdlog::info("successfully written {} points", p_count);

    if (laszip_close_writer(laszip_writer))
    {
        spdlog::error("DLL ERROR: closing laszip writer");
        return false;
    }

    if (laszip_destroy(laszip_writer))
    {
        spdlog::error("DLL ERROR: destroying laszip writer");
        return false;
    }

    std::cout << "exportLaz DONE" << std::endl;
    return true;
}
