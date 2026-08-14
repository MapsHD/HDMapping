#include "RosExport.h"
#include "TrajectoryViewerShaders.h"
#include "external/glad.h"
#include "imgui.h"
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "rlgl.h"
#include <CalibCore/Camera.h>
#include <CalibCore/CliArgs.h>
#include <CalibCore/PointCloud.h>
#include <CalibCore/Trajectory.h>
#include <Core/pfd_wrapper.hpp>
#include <HDMapping/PoseInterpolation.h>
#include <HDMapping/Version.hpp>
#include <RaylibWidgets/CenterOfRotationWindow.h>
#include <RaylibWidgets/CompassRuler.h>
#include <RaylibWidgets/OrbitCamera.h>
#include <RaylibWidgets/ShortcutsTable.h>
#include <RaylibWidgets/WindowFit.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <laszip/laszip_api.h>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
using namespace calib;
namespace fs = std::filesystem;

// Shortcuts help table (Help menu). Only lists this app's actual bindings --
// no A-Z scaffold like multi_view_tls_registration_step_2's, since
// ShowShortcutsTable() just renders whatever it's given.
static const std::vector<raylib_widgets::ShortcutEntry> appShortcuts = {
    { "Normal keys", "C", "Toggle compass/ruler" },
    { "", "P", "Toggle show path" },
    { "", "V", "Toggle show frustums" },
    { "", "Ctrl (tap)", "Toggle Intensity <-> RGB point color" },
    { "", "Ctrl+O", "Select LIO result directory" },
    { "", "Ctrl+Shift+O", "Select CAMERA_0 directory" },
    { "", "Ctrl+Shift+C", "Open calibration" },
    { "", "Ctrl+S", "Export colored point cloud" },
    { "Special keys", "Left arrow", "Previous image (image preview)" },
    { "", "Right arrow", "Next image (image preview)" },
    { "Mouse related", "Left click + drag", "Orbit camera" },
    { "", "Right click + drag", "Pan camera" },
    { "", "Scroll", "Zoom camera" },
    { "", "Ctrl+Right click", "Set center of rotation (ground plane)" },
    { "", "Middle click", "Set center of rotation (ground plane)" },
    { "", "Shift+R", "Open 'Center of rotation' dialog" },
};

// Copies `path` into `buf` (truncating to fit), for wiring a native-dialog
// result back into the same fixed-size char[] the matching text field edits.
static void setBuf(char* buf, size_t bufSize, const std::string& path)
{
    if (path.empty())
        return;
    std::strncpy(buf, path.c_str(), bufSize - 1);
    buf[bufSize - 1] = '\0';
}

// Build a time(seconds) -> T_world_lidar map suitable for getInterpolatedPose().
static std::map<double, Eigen::Matrix4d> buildTrajMap(const Trajectory& traj)
{
    std::map<double, Eigen::Matrix4d> m;
    for (const auto& p : traj.poses)
        m[p.ts_ns * 1e-9] = p.T.matrix().cast<double>();
    return m;
}

// Interpolated T_world_lidar at ts_ns. Returns false when ts_ns lies outside the
// trajectory range — getInterpolatedPose() signals that with a zero matrix.
static bool interpPose(const std::map<double, Eigen::Matrix4d>& trajMap, int64_t ts_ns, Eigen::Affine3f& out)
{
    Eigen::Matrix4d T = getInterpolatedPose(trajMap, ts_ns * 1e-9);
    if (T(3, 3) == 0.0)
        return false;
    out.matrix() = T.cast<float>();
    return true;
}

using trajectory_viewer_shaders::kFS;
using trajectory_viewer_shaders::kVS;

struct GpuCloud
{
    unsigned int vao = 0, vbo = 0;
    int count = 0;
    float maxDist = 50.f;

    void upload(const std::vector<float>& data, float mx)
    {
        unload();
        if (data.empty())
            return;
        maxDist = mx;
        vao = rlLoadVertexArray();
        rlEnableVertexArray(vao);
        vbo = rlLoadVertexBuffer(data.data(), (int)(data.size() * sizeof(float)), false);
        const int stride = 7 * sizeof(float);
        rlSetVertexAttribute(0, 3, RL_FLOAT, false, stride, 0);
        rlEnableVertexAttribute(0);
        rlSetVertexAttribute(1, 1, RL_FLOAT, false, stride, 3 * sizeof(float));
        rlEnableVertexAttribute(1);
        rlSetVertexAttribute(2, 1, RL_FLOAT, false, stride, 4 * sizeof(float));
        rlEnableVertexAttribute(2);
        rlSetVertexAttribute(3, 1, RL_FLOAT, false, stride, 5 * sizeof(float));
        rlEnableVertexAttribute(3);
        rlSetVertexAttribute(4, 1, RL_FLOAT, false, stride, 6 * sizeof(float));
        rlEnableVertexAttribute(4);
        rlDisableVertexArray();
        count = (int)(data.size() / 7);
    }
    void unload()
    {
        if (vao)
        {
            rlUnloadVertexArray(vao);
            vao = 0;
        }
        if (vbo)
        {
            rlUnloadVertexBuffer(vbo);
            vbo = 0;
        }
        count = 0;
    }
};

struct ColorPt
{
    float x, y, z;
    uint8_t r, g, b;
    float intensity;
    int64_t ts_ns;
};

// ── Application state ─────────────────────────────────────────────────────────
struct AppState
{
    Trajectory traj;
    std::vector<int64_t> imageTsNs;
    Intrinsics K;
    Extrinsics E; // tx/ty/tz (camera position); rotation lives in R_wc below, not E.om/fi/ka
    Eigen::Matrix3f R_wc = Eigen::Matrix3f::Identity(); // camera orientation in world/LiDAR frame
    Roi roi;
    bool calibLoaded = false;
    int imgW = 4656, imgH = 3496;

    // loaded camera images: timestamp → resized BGR Mat
    std::map<int64_t, std::string> imagesFilenamesInTime;
    const float imgScale = 1.0f;
    GpuCloud cloud;
    Shader shader = {};
    bool shaderOk = false;
    int locMVP = -1, locPS = -1, locCM = -1, locDecim = -1, locSel = -1;

    raylib_widgets::OrbitCamera orbit;
    bool showCenterOfRotationWindow = false;

    // controls
    bool showPath = true;
    bool showFrustums = true;
    bool showCompassRuler = true;
    bool showHelp = false;
    bool isolateCamera = false; // render only points colored by the selected (preview) image
    float frustumScale = 0.5f;
    float pointSize = 2.f;
    int cloudDecim = 5;
    int drawDecim = 1;
    bool multiImgColoring = true; // false = single image per chunk (midpoint)
    // How each point is matched to a camera image:
    //   0 = temporal  — image nearest in time (± maxWiggle frames, within maxTemporalDist)
    //   1 = geometry  — among all chunk images the point projects into, the one
    //                   with the smallest depth (closest camera)
    int colorStrategy = 0;
    float maxTemporalDist = 0.5f; // s: skip images farther than this from the point (temporal)
    int maxWiggle = 1; // frames: search startIdx ± maxWiggle for a frustum hit (temporal)
    bool useImageColor = false; // true once a colorize pass produced RGB data
    int colorMode = 0; // 0=intensity (jet), 1=RGB by image, 2=camera id
    int coloredPts = 0; // points that received RGB from an image
    int uncoloredPts = 0; // points left as intensity-gray (no image / out of frustum / outside ROI)

    char sessionBuf[512] = {};
    char calibBuf[512] = {};
    char cameraBuf[512] = {};
    char exportBuf[512] = "colored.laz";
    std::vector<ColorPt> exportCloud;
    std::string status;

    // ── ROS 2 export ──────────────────────────────────────────────────────────
    char rosOutBuf[512] = "ros2_export";
    int rosStorageIdx = 0; // 0 = mcap, 1 = sqlite3
    RosExportOptions ros;
    std::thread rosThread;
    std::atomic<bool> rosBusy{ false };
    std::mutex rosMtx;
    std::string rosResult;
    bool rosResultReady = false;

    // ── COLMAP export ─────────────────────────────────────────────────────────
    char colmapBuf[512] = "colmap_out";
    bool colmapCopyImages = false;
    int colmapPtDecim = 50; // splat-friendly default (~500k from a 25M cloud)

    // ── image viewer ────────────────────────────────────────────────────────
    int imgViewIdx = 0;
    Texture2D imgViewTex = {};
    bool imgViewTexValid = false;
    std::atomic<int> imgViewRequest{ -1 };
    std::atomic<bool> imgViewStop{ false };
    std::atomic<bool> imgViewLoading{ false };
    std::mutex imgViewMtx;
    cv::Mat imgViewPending;
    bool imgViewHasNew = false;
    std::thread imgViewThread;
};

// ── helpers ───────────────────────────────────────────────────────────────────
static Vector3 toRL(float x, float y, float z)
{
    return { x, z, -y };
}
static Vector3 toRL(const Eigen::Vector3f& v)
{
    return { v.x(), v.z(), -v.y() };
}

// Load all cam0_*.jpg from CAMERA_0 (sibling of session dir) into s.images, resized by s.imgScale.
static void loadImages(AppState& s)
{
    s.imagesFilenamesInTime.clear();
    fs::path camDir;
    if (s.cameraBuf[0])
    {
        camDir = fs::path(s.cameraBuf);
    }
    else
    {
        camDir = fs::path(s.sessionBuf).parent_path() / "CAMERA_0";
    }
    if (!fs::is_directory(camDir))
    {
        s.status = "No CAMERA_0 dir found";
        return;
    }

    int loaded = 0;
    for (auto& e : fs::directory_iterator(camDir))
    {
        std::string n = e.path().filename().string();
        if (n.rfind("cam0_", 0) != 0 || e.path().extension() != ".jpg")
            continue;
        try
        {
            // filename: cam0_<timestamp_ns>.jpg  → strip prefix (5) and ext (4)
            int64_t ts = std::stoll(n.substr(5, n.size() - 9));
            s.imagesFilenamesInTime[ts] = e.path().string();
            ++loaded;
        } catch (...)
        {
        }
    }
    s.status = "Images loaded: " + std::to_string(loaded) + " from " + camDir.string();
}

// Parse session_poses.mrp → map from chunk stem (e.g. "scan_lio_0") to Affine3f.
static std::map<std::string, Eigen::Affine3f> parseMRP(const fs::path& mrpPath)
{
    std::map<std::string, Eigen::Affine3f> result;
    std::ifstream f(mrpPath);
    if (!f)
        return result;
    int n;
    f >> n;
    for (int i = 0; i < n; i++)
    {
        std::string name;
        f >> name;
        auto dot = name.rfind('.');
        std::string key = (dot != std::string::npos) ? name.substr(0, dot) : name;
        float raw[16];
        for (int r = 0; r < 16; r++)
            f >> raw[r];
        if (!f)
            continue;
        Eigen::Matrix4f M4;
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                M4(r, c) = raw[r * 4 + c];
        result[key] = Eigen::Affine3f(M4);
    }
    return result;
}

static void loadSession(AppState& s)
{
    s.traj.poses.clear();
    s.imageTsNs.clear();
    s.exportCloud.clear();
    s.cloud.unload();
    loadImages(s);

    fs::path d(s.sessionBuf);
    if (!fs::is_directory(d))
    {
        s.status = "Not a directory";
        return;
    }

    // parse MRP if present
    auto mrp = parseMRP(d / "session_poses.mrp");
    if (mrp.empty())
        mrp = parseMRP(d / "session_ini_poses.mri");

    // trajectory CSVs — apply corresponding MRP transform per chunk
    std::vector<fs::path> csvPaths;
    for (auto& e : fs::directory_iterator(d))
    {
        std::string n = e.path().filename().string();
        if (n.rfind("trajectory_lio_", 0) == 0 && e.path().extension() == ".csv")
            csvPaths.push_back(e.path());
    }
    std::sort(csvPaths.begin(), csvPaths.end());
    for (auto& cp : csvPaths)
    {
        std::string stem = cp.stem().string();
        std::string idx = stem.substr(stem.rfind('_') + 1);
        std::string key = "scan_lio_" + idx;
        const Eigen::Affine3f* M = mrp.count(key) ? &mrp.at(key) : nullptr;
        s.traj.loadCSV(cp.string(), M);
    }
    s.traj.sort();

    // camera image timestamps
    fs::path camDir = s.cameraBuf[0] ? fs::path(s.cameraBuf) : d.parent_path() / "CAMERA_0";
    if (fs::is_directory(camDir))
    {
        for (auto& e : fs::directory_iterator(camDir))
        {
            std::string n = e.path().filename().string();
            if (n.rfind("cam0_", 0) == 0 && e.path().extension() == ".jpg")
            {
                try
                {
                    int64_t ts = std::stoll(n.substr(5, n.size() - 9));
                    s.imageTsNs.push_back(ts);
                } catch (...)
                {
                }
            }
        }
        std::sort(s.imageTsNs.begin(), s.imageTsNs.end());
    }

    s.status = "Poses: " + std::to_string(s.traj.poses.size()) + "  Img: " + std::to_string(s.imageTsNs.size()) +
        (mrp.empty() ? "  (no MRP)" : "  +MRP") + "  — press Load cloud";
}

static void loadCloud(AppState& s)
{
    s.exportCloud.clear();
    s.cloud.unload();

    fs::path d(s.sessionBuf);
    if (!fs::is_directory(d))
    {
        s.status = "No session loaded";
        return;
    }

    auto mrp = parseMRP(d / "session_poses.mrp");
    if (mrp.empty())
        mrp = parseMRP(d / "session_ini_poses.mri");

    std::vector<fs::path> lazPaths;
    for (auto& e : fs::directory_iterator(d))
    {
        std::string n = e.path().filename().string();
        if (n.rfind("scan_lio_", 0) == 0 && e.path().extension() == ".laz")
            lazPaths.push_back(e.path());
    }
    std::sort(lazPaths.begin(), lazPaths.end());

    bool canColor = s.calibLoaded && !s.imagesFilenamesInTime.empty();
    Eigen::Matrix3f R_wc = canColor ? s.R_wc : Eigen::Matrix3f::Identity();
    Eigen::Vector3f C(s.E.tx, s.E.ty, s.E.tz);
    float K_fx = s.K.fx * s.imgScale, K_fy = s.K.fy * s.imgScale;
    float K_cx = s.K.cx * s.imgScale, K_cy = s.K.cy * s.imgScale;
    // OpenCV rational + tangential distortion applied to each projected point, so
    // colours are sampled from the raw (distorted) images at the right pixel.
    // With all-zero coefficients this reduces exactly to the pinhole model.
    const float d_k1 = s.K.k1, d_k2 = s.K.k2, d_k3 = s.K.k3;
    const float d_k4 = s.K.k4, d_k5 = s.K.k5, d_k6 = s.K.k6;
    const float d_p1 = s.K.p1, d_p2 = s.K.p2;
    // (x, y) = normalized camera coords (X/Z, Y/Z) → distorted normalized coords.
    auto distort = [=](float x, float y, float& xd, float& yd)
    {
        float r2 = x * x + y * y;
        float radial = (1.f + (d_k1 + (d_k2 + d_k3 * r2) * r2) * r2) / (1.f + (d_k4 + (d_k5 + d_k6 * r2) * r2) * r2);
        xd = x * radial + 2.f * d_p1 * x * y + d_p2 * (r2 + 2.f * x * x);
        yd = y * radial + d_p1 * (r2 + 2.f * y * y) + 2.f * d_p2 * x * y;
    };

    auto packGray = [](float intensity) -> float
    {
        uint8_t g = (uint8_t)(std::min(1.f, std::max(0.f, intensity)) * 255.f);
        uint32_t p = (uint32_t(g) << 16) | (uint32_t(g) << 8) | uint32_t(g);
        float f;
        std::memcpy(&f, &p, 4);
        return f;
    };

    struct ImgEntry
    {
        int64_t ts;
        Eigen::Affine3f pose; // T_world_lidar at the image time (interpolated)
        cv::Mat img;
        int globalIdx; // index into s.imageTsNs (== imgViewIdx / selectedCamera)
    };

    // time(s) -> T_world_lidar, for interpolating the pose at each image time.
    std::map<double, Eigen::Matrix4d> trajMap = buildTrajMap(s.traj);

    std::vector<float> gpuData;
    float mx = 0.f;
    float sumX = 0, sumY = 0, sumZ = 0;
    int cnt = 0;
    int step = std::max(1, s.cloudDecim);
    int coloredChunks = 0;
    int coloredPts = 0, uncoloredPts = 0;

    for (auto& lp : lazPaths)
    {
        std::string key = lp.stem().string(); // "scan_lio_N"
        std::string idx = key.substr(key.rfind('_') + 1);
        const Eigen::Affine3f* M = mrp.count(key) ? &mrp.at(key) : nullptr;

        // ── step 1: read chunk time range from the matching trajectory CSV ──
        int64_t chunkFirst = 0, chunkLast = 0;
        {
            fs::path csvPath = d / ("trajectory_lio_" + idx + ".csv");
            std::ifstream cf(csvPath);
            if (cf)
            {
                std::string line;
                std::getline(cf, line);
                while (std::getline(cf, line))
                {
                    if (line.empty())
                        continue;
                    std::istringstream ss(line);
                    int64_t ts;
                    ss >> ts;
                    if (!ss)
                        continue;
                    if (!chunkFirst)
                        chunkFirst = ts;
                    chunkLast = ts;
                }
            }
        }

        // ── step 2: collect images for this chunk ───────────────────────────
        std::vector<ImgEntry> chunkImgs;
        if (canColor && chunkFirst && chunkLast)
        {
            if (s.multiImgColoring)
            {
                // new: every image whose timestamp falls inside the chunk range
                auto it0 = std::lower_bound(s.imageTsNs.begin(), s.imageTsNs.end(), chunkFirst);
                auto it1 = std::upper_bound(s.imageTsNs.begin(), s.imageTsNs.end(), chunkLast);
                for (auto it = it0; it != it1; ++it)
                {
                    int64_t imgTs = *it;
                    auto fnIt = s.imagesFilenamesInTime.find(imgTs);
                    if (fnIt == s.imagesFilenamesInTime.end())
                        continue;
                    Eigen::Affine3f pose;
                    if (!interpPose(trajMap, imgTs, pose))
                        continue;
                    cv::Mat img = cv::imread(fnIt->second);
                    if (img.empty())
                        continue;
                    int gidx = (int)(it - s.imageTsNs.begin());
                    chunkImgs.push_back({ imgTs, pose, std::move(img), gidx });
                }
            }
            else
            {
                // legacy: single image nearest to chunk midpoint
                int64_t mid = chunkFirst;
                auto it = std::lower_bound(s.imageTsNs.begin(), s.imageTsNs.end(), mid);
                if (it == s.imageTsNs.end())
                    --it;
                else if (it != s.imageTsNs.begin())
                {
                    auto prev = std::prev(it);
                    if (std::abs(*prev - mid) < std::abs(*it - mid))
                        it = prev;
                }
                int64_t imgTs = *it;
                auto fnIt = s.imagesFilenamesInTime.find(imgTs);
                Eigen::Affine3f pose;
                if (fnIt != s.imagesFilenamesInTime.end() && interpPose(trajMap, imgTs, pose))
                {
                    cv::Mat img = cv::imread(fnIt->second);
                    int gidx = (int)(it - s.imageTsNs.begin());
                    if (!img.empty())
                        chunkImgs.push_back({ imgTs, pose, std::move(img), gidx });
                }
            }
        }
        if (!chunkImgs.empty())
            ++coloredChunks;

        // ── step 3: load point cloud ────────────────────────────────────────
        PointCloud pc;
        if (!pc.load(lp.string()))
            continue;

        int nImgs = (int)chunkImgs.size();

        // ── step 4: colorize each point ─────────────────────────────────────
        // chunkImgs is sorted by ts (imageTsNs was sorted)
        // For each point: find nearest image by pt.ts_ns, expand outward until
        // the point lands inside a frustum.
        for (int i = 0; i < (int)pc.points.size(); i += step)
        {
            auto& pt = pc.points[i];
            Eigen::Vector3f pw(pt.x, pt.y, pt.z);
            if (M)
                pw = *M * pw;

            gpuData.push_back(pw.x());
            gpuData.push_back(pw.z());
            gpuData.push_back(-pw.y());

            const float rawIntensity = pt.intensity;
            float colorF = packGray(rawIntensity);
            float camIdF = -1.f; // which image colored this point (global index), -1 = none
            float inRoiF = -1.f; // 1=inside ROI, 0=outside ROI, -1=projects into no image

            if (nImgs > 0)
            {
                // nearest image by point timestamp
                int startIdx = 0;
                if (pt.ts_ns != 0)
                {
                    auto it = std::lower_bound(
                        chunkImgs.begin(),
                        chunkImgs.end(),
                        pt.ts_ns,
                        [](const ImgEntry& e, int64_t t)
                        {
                            return e.ts < t;
                        });
                    if (it == chunkImgs.end())
                        --it;
                    else if (it != chunkImgs.begin())
                    {
                        auto prev = std::prev(it);
                        if (std::abs(prev->ts - pt.ts_ns) < std::abs(it->ts - pt.ts_ns))
                            it = prev;
                    }
                    startIdx = (int)(it - chunkImgs.begin());
                }
                // Result of projecting the point into one image.
                struct Hit
                {
                    bool ok = false; // projects into frustum AND passes ROI filter
                    float depth = 0.f; // z in camera frame (only when ok)
                    float colorF = 0.f; // packed RGB (only when ok)
                    float inRoiF = -1.f; // 1 inside ROI, 0 outside, -1 not in frustum
                    int globalIdx = -1;
                };
                auto probe = [&](int idx) -> Hit
                {
                    Hit h;
                    if (idx < 0 || idx >= nImgs)
                        return h;
                    auto& e = chunkImgs[idx];
                    Eigen::Vector3f pl = e.pose.inverse() * pw;
                    Eigen::Vector3f pc_ = R_wc.transpose() * (pl - C);
                    if (pc_.z() <= 0.05f)
                        return h;
                    float xd, yd;
                    distort(pc_.x() / pc_.z(), pc_.y() / pc_.z(), xd, yd);
                    int iu = (int)std::round(K_fx * xd + K_cx);
                    int iv = (int)std::round(K_fy * yd + K_cy);
                    if (iu < 0 || iu >= e.img.cols || iv < 0 || iv >= e.img.rows)
                        return h;
                    // point projects into this image — record ROI membership so
                    // the "In ROI" render mode can show it, independent of whether
                    // the ROI filter is currently enabled.
                    bool haveRoi = s.roi.w > 0 && s.roi.h > 0;
                    bool insideRoi = !haveRoi || (iu >= s.roi.x && iu < s.roi.x + s.roi.w && iv >= s.roi.y && iv < s.roi.y + s.roi.h);
                    h.inRoiF = insideRoi ? 1.f : 0.f;
                    // outside the region of interest? leave the point uncolored
                    if (s.roi.enabled && !insideRoi)
                        return h;
                    cv::Vec3b bgr = e.img.at<cv::Vec3b>(iv, iu);
                    uint32_t p = (uint32_t(bgr[2]) << 16) | (uint32_t(bgr[1]) << 8) | uint32_t(bgr[0]);
                    std::memcpy(&h.colorF, &p, 4);
                    h.globalIdx = e.globalIdx;
                    h.depth = pc_.z();
                    h.ok = true;
                    return h;
                };
                // Record frustum/ROI membership even for images that don't win, so
                // the "In ROI" render mode stays meaningful. Latest wins.
                auto note = [&](const Hit& h)
                {
                    if (h.inRoiF >= 0.f)
                        inRoiF = h.inRoiF;
                };
                auto commit = [&](const Hit& h)
                {
                    colorF = h.colorF;
                    camIdF = (float)h.globalIdx;
                    inRoiF = h.inRoiF;
                };

                if (s.colorStrategy == 1)
                {
                    // Geometry: among every image the point projects into, keep the
                    // one with the smallest depth (closest camera → best resolution).
                    Hit best;
                    for (int idx = 0; idx < nImgs; ++idx)
                    {
                        Hit h = probe(idx);
                        note(h);
                        if (h.ok && (!best.ok || h.depth < best.depth))
                            best = h;
                    }
                    if (best.ok)
                        commit(best);
                }
                else
                {
                    // Temporal: search the temporally-nearest image, then its
                    // neighbours outward (±1, ±2, … ±maxWiggle), taking the first
                    // frustum hit. Only if the nearest image is within
                    // maxTemporalDist of the point.
                    const int64_t maxDtNs = (int64_t)(s.maxTemporalDist * 1e9);
                    if (std::abs(pt.ts_ns - chunkImgs[startIdx].ts) <= maxDtNs)
                    {
                        for (int w = 0; w <= s.maxWiggle && camIdF < 0.f; ++w)
                        {
                            Hit h = probe(startIdx - w);
                            note(h);
                            if (h.ok)
                            {
                                commit(h);
                                break;
                            }
                            if (w == 0)
                                continue;
                            h = probe(startIdx + w);
                            note(h);
                            if (h.ok)
                            {
                                commit(h);
                                break;
                            }
                        }
                    }
                }
            }
            if (camIdF >= 0.f)
                ++coloredPts;
            else
                ++uncoloredPts;

            gpuData.push_back(colorF);
            gpuData.push_back(rawIntensity);
            gpuData.push_back(camIdF);
            gpuData.push_back(inRoiF);

            uint32_t packed;
            std::memcpy(&packed, &colorF, 4);
            s.exportCloud.push_back(
                { pw.x(),
                  pw.y(),
                  pw.z(),
                  (uint8_t)((packed >> 16) & 0xFF),
                  (uint8_t)((packed >> 8) & 0xFF),
                  (uint8_t)(packed & 0xFF),
                  rawIntensity,
                  pt.ts_ns });

            float d2 = pw.squaredNorm();
            if (d2 > mx * mx)
                mx = std::sqrt(d2);
            sumX += pw.x();
            sumY += pw.z();
            sumZ += -pw.y();
            cnt++;
        }
        // chunkImgs and their cv::Mat memory are released here
    }
    s.useImageColor = canColor && (coloredChunks > 0);
    if (s.useImageColor)
        s.colorMode = 1; // default to RGB display once RGB data is available
    s.coloredPts = coloredPts;
    s.uncoloredPts = uncoloredPts;

    if (cnt > 0)
    {
        s.cloud.upload(gpuData, mx);
        s.orbit.target = { sumX / cnt, sumY / cnt, sumZ / cnt };
        s.orbit.distance = std::max(5.f, mx * 0.3f);
    }

    s.status = "Pts: " + std::to_string(s.cloud.count) + "  Poses: " + std::to_string(s.traj.poses.size()) +
        "  Imgs/chunk: " + std::to_string(coloredChunks > 0 ? coloredChunks : 0) + (s.useImageColor ? "  +RGB" : "");
    if (s.useImageColor && cnt > 0)
    {
        double pct = 100.0 * coloredPts / cnt;
        s.status += "  | Colored: " + std::to_string(coloredPts) + "  Uncolored: " + std::to_string(uncoloredPts) + "  (" +
            std::to_string((int)std::lround(pct)) + "%)";
    }
}

static void loadCalib(AppState& s)
{
    std::ifstream f(s.calibBuf);
    if (!f)
    {
        s.status = std::string("Cannot open: ") + s.calibBuf;
        return;
    }
    nlohmann::json j;
    f >> j;
    if (j.contains("intrinsics"))
    {
        auto& ji = j["intrinsics"];
        s.K.fx = ji.value("fx", s.K.fx);
        s.K.fy = ji.value("fy", s.K.fy);
        s.K.cx = ji.value("cx", s.K.cx);
        s.K.cy = ji.value("cy", s.K.cy);
        // rational distortion model (used by ROS export to rectify images)
        s.K.k1 = ji.value("k1", s.K.k1);
        s.K.k2 = ji.value("k2", s.K.k2);
        s.K.k3 = ji.value("k3", s.K.k3);
        s.K.k4 = ji.value("k4", s.K.k4);
        s.K.k5 = ji.value("k5", s.K.k5);
        s.K.k6 = ji.value("k6", s.K.k6);
        s.K.p1 = ji.value("p1", s.K.p1);
        s.K.p2 = ji.value("p2", s.K.p2);
    }
    if (j.contains("extrinsics"))
    {
        auto& je = j["extrinsics"];
        if (je.contains("camera_position_in_world_xyz") && je["camera_position_in_world_xyz"].size() >= 3)
        {
            s.E.tx = je["camera_position_in_world_xyz"][0];
            s.E.ty = je["camera_position_in_world_xyz"][1];
            s.E.tz = je["camera_position_in_world_xyz"][2];
        }
        // camera_rotation_matrix_in_world: 3x3 rows of R_wc, read straight into
        // s.R_wc -- the viewer only ever consumes this calibration, so there's
        // no need to round-trip it through Tait-Bryan angles.
        if (je.contains("camera_rotation_matrix_in_world") && je["camera_rotation_matrix_in_world"].size() >= 3)
        {
            auto& m = je["camera_rotation_matrix_in_world"];
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    s.R_wc(r, c) = m[r][c].get<float>();
        }
    }
    // Optional region of interest, in full-resolution image pixels:
    //   "roi": { "x": 0, "y": 0, "w": 4656, "h": 3496, "enabled": true }
    // "enabled" defaults to true when the object is present; it only takes
    // effect once w and h are positive.
    if (j.contains("roi"))
    {
        auto& jr = j["roi"];
        s.roi.x = jr.value("x", 0);
        s.roi.y = jr.value("y", 0);
        s.roi.w = jr.value("w", 0);
        s.roi.h = jr.value("h", 0);
        s.roi.enabled = jr.value("enabled", true) && s.roi.w > 0 && s.roi.h > 0;
    }
    s.calibLoaded = true;
    s.status = "Calibration loaded";
}

static void exportLAZ(AppState& s)
{
    if (s.exportCloud.empty())
    {
        s.status = "No cloud to export";
        return;
    }

    double xmin = s.exportCloud[0].x, xmax = xmin;
    double ymin = s.exportCloud[0].y, ymax = ymin;
    double zmin = s.exportCloud[0].z, zmax = zmin;
    for (auto& p : s.exportCloud)
    {
        xmin = std::min(xmin, (double)p.x);
        xmax = std::max(xmax, (double)p.x);
        ymin = std::min(ymin, (double)p.y);
        ymax = std::max(ymax, (double)p.y);
        zmin = std::min(zmin, (double)p.z);
        zmax = std::max(zmax, (double)p.z);
    }

    laszip_POINTER writer = nullptr;
    if (laszip_create(&writer))
    {
        s.status = "laszip_create failed";
        return;
    }

    laszip_header* header = nullptr;
    laszip_get_header_pointer(writer, &header);

    header->version_major = 1;
    header->version_minor = 2;
    header->header_size = 227;
    header->offset_to_point_data = 227;
    header->point_data_format = 3; // XYZ + RGB + GPS time
    header->point_data_record_length = 34;
    header->number_of_point_records = (uint32_t)s.exportCloud.size();
    header->x_scale_factor = 0.001;
    header->y_scale_factor = 0.001;
    header->z_scale_factor = 0.001;
    header->x_offset = xmin;
    header->y_offset = ymin;
    header->z_offset = zmin;
    header->min_x = xmin;
    header->max_x = xmax;
    header->min_y = ymin;
    header->max_y = ymax;
    header->min_z = zmin;
    header->max_z = zmax;

    laszip_BOOL compress = (std::strstr(s.exportBuf, ".laz") != nullptr) ? 1 : 0;
    if (laszip_open_writer(writer, s.exportBuf, compress))
    {
        laszip_CHAR* err = nullptr;
        laszip_get_error(writer, &err);
        s.status = std::string("Export failed: ") + (err ? err : "?");
        laszip_destroy(writer);
        return;
    }

    laszip_point* point = nullptr;
    laszip_get_point_pointer(writer, &point);

    laszip_F64 coords[3];
    for (auto& p : s.exportCloud)
    {
        coords[0] = p.x;
        coords[1] = p.y;
        coords[2] = p.z;
        laszip_set_coordinates(writer, coords);
        point->rgb[0] = (laszip_U16)p.r << 8;
        point->rgb[1] = (laszip_U16)p.g << 8;
        point->rgb[2] = (laszip_U16)p.b << 8;
        // intensity normalized [0,1] → LAS 16-bit field
        point->intensity = (laszip_U16)(std::min(1.f, std::max(0.f, p.intensity)) * 65535.f);
        // GPS time: ns since epoch → seconds (double)
        point->gps_time = (laszip_F64)p.ts_ns * 1e-9;
        laszip_write_point(writer);
    }

    laszip_close_writer(writer);
    laszip_destroy(writer);
    s.status = "Exported " + std::to_string(s.exportCloud.size()) + " pts → " + s.exportBuf;
}

// ── File actions ─────────────────────────────────────────────────────────────
// Factored out so the File menu items and their keyboard shortcuts (in the
// main loop below) call the exact same code, matching the openSession()-style
// convention used by mandeye_single_session_viewer/multi_view_tls_registration.
static void actionSelectLioResultDir(AppState& s)
{
    setBuf(s.sessionBuf, sizeof(s.sessionBuf), mandeye::fd::SelectFolder("Select LIO result directory"));
}

static void actionSelectCamera0Dir(AppState& s)
{
    setBuf(s.cameraBuf, sizeof(s.cameraBuf), mandeye::fd::SelectFolder("Select CAMERA_0 directory"));
}

static void actionOpenCalibration(AppState& s)
{
    std::string path = mandeye::fd::OpenFileDialogOneFile("Select calibration file", mandeye::fd::json_filter);
    if (!path.empty())
    {
        setBuf(s.calibBuf, sizeof(s.calibBuf), path);
        loadCalib(s);
    }
}

static void actionExportColoredPointCloud(AppState& s)
{
    std::string defaultName = fs::path(s.exportBuf).filename().string();
    std::string path = mandeye::fd::SaveFileDialog("Export colored point cloud", mandeye::fd::LazFilter, ".laz", defaultName);
    if (!path.empty())
    {
        setBuf(s.exportBuf, sizeof(s.exportBuf), path);
        exportLAZ(s);
    }
}

static void actionSelectRosOutputDir(AppState& s)
{
    setBuf(s.rosOutBuf, sizeof(s.rosOutBuf), mandeye::fd::SelectFolder("Select ROS 2 bag output directory"));
}

static void actionSelectColmapOutputDir(AppState& s)
{
    setBuf(s.colmapBuf, sizeof(s.colmapBuf), mandeye::fd::SelectFolder("Select COLMAP output directory"));
}

// Export a COLMAP sparse text model (cameras/images/points3D) from the current
// state. Poses are world->camera; the colored cloud becomes points3D.
static void exportColmap(AppState& s)
{
    if (!s.calibLoaded)
    {
        s.status = "COLMAP: load calibration first";
        return;
    }
    if (s.imagesFilenamesInTime.empty())
    {
        s.status = "COLMAP: no images";
        return;
    }

    fs::path out(s.colmapBuf);
    fs::path sparse = out / "sparse";
    std::error_code ec;
    fs::create_directories(sparse, ec);
    if (ec)
    {
        s.status = "COLMAP: cannot create " + sparse.string();
        return;
    }

    // T_lidar_camera (camera pose in the LiDAR frame, from the extrinsics)
    Eigen::Affine3f T_lc = Eigen::Affine3f::Identity();
    T_lc.linear() = s.R_wc;
    T_lc.translation() = Eigen::Vector3f(s.E.tx, s.E.ty, s.E.tz);

    // cameras.txt — rational OpenCV model == COLMAP FULL_OPENCV (12 params)
    {
        std::ofstream f(sparse / "cameras.txt");
        f << std::setprecision(12);
        f << "# Camera list with one line of data per camera:\n"
             "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
        f << "1 FULL_OPENCV " << s.imgW << ' ' << s.imgH << ' ' << s.K.fx << ' ' << s.K.fy << ' ' << s.K.cx << ' ' << s.K.cy << ' '
          << s.K.k1 << ' ' << s.K.k2 << ' ' << s.K.p1 << ' ' << s.K.p2 << ' ' << s.K.k3 << ' ' << s.K.k4 << ' ' << s.K.k5 << ' ' << s.K.k6
          << '\n';
    }

    // images.txt — one image per camera frame, pose = world->camera
    int nImg = 0;
    {
        std::ofstream f(sparse / "images.txt");
        f << std::setprecision(12);
        f << "# Image list with two lines of data per image:\n"
             "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n"
             "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
        auto trajMap = buildTrajMap(s.traj);
        int id = 1;
        for (auto& [ts, path] : s.imagesFilenamesInTime)
        {
            Eigen::Affine3f pose;
            if (!interpPose(trajMap, ts, pose))
                continue;
            Eigen::Affine3f T_wc = pose * T_lc; // camera in world
            Eigen::Affine3f T_cw = T_wc.inverse(); // world -> camera
            Eigen::Quaternionf q(T_cw.linear());
            q.normalize();
            Eigen::Vector3f t = T_cw.translation();
            std::string name = fs::path(path).filename().string();
            f << id << ' ' << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << " 1 "
              << name << '\n';
            f << '\n'; // empty POINTS2D line (no 2D-3D correspondences)
            ++id;
            ++nImg;
        }
    }

    // points3D.txt — the colored cloud (no tracks)
    size_t nPts = 0;
    {
        std::ofstream f(sparse / "points3D.txt");
        f << "# 3D point list with one line of data per point:\n"
             "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n";
        f << std::setprecision(9);
        int step = std::max(1, s.colmapPtDecim);
        size_t id = 1;
        for (size_t i = 0; i < s.exportCloud.size(); i += step)
        {
            const auto& p = s.exportCloud[i];
            f << id << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' ' << (int)p.r << ' ' << (int)p.g << ' ' << (int)p.b << " 0\n";
            ++id;
            ++nPts;
        }
    }

    // points3D.ply — binary PLY (xyz + uchar rgb), same decimation. Convenient
    // init cloud for 3DGS trainers and opens directly in CloudCompare.
    {
        int step = std::max(1, s.colmapPtDecim);
        size_t n = (s.exportCloud.size() + step - 1) / step;
        std::ofstream f(sparse / "points3D.ply", std::ios::binary);
        f << "ply\nformat binary_little_endian 1.0\n"
          << "element vertex " << n << "\n"
          << "property float x\nproperty float y\nproperty float z\n"
          << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
          << "end_header\n";
        for (size_t i = 0; i < s.exportCloud.size(); i += step)
        {
            const auto& p = s.exportCloud[i];
            f.write(reinterpret_cast<const char*>(&p.x), sizeof(float) * 3);
            f.write(reinterpret_cast<const char*>(&p.r), 3); // r,g,b contiguous
        }
    }

    if (s.colmapCopyImages)
    {
        fs::path imgd = out / "images";
        fs::create_directories(imgd, ec);
        for (auto& [ts, path] : s.imagesFilenamesInTime)
            fs::copy_file(path, imgd / fs::path(path).filename(), fs::copy_options::overwrite_existing, ec);
    }

    s.status = "COLMAP: " + std::to_string(nImg) + " images, " + std::to_string(nPts) + " points (+ply) -> " + sparse.string();
}

// Gather everything the ROS exporter needs from current viewer state.
static void buildRosInput(AppState& s, RosExportInput& in)
{
    in.traj = s.traj;
    in.imageFiles = s.imagesFilenamesInTime;
    in.calibLoaded = s.calibLoaded;
    in.K = s.K;
    in.E = s.E;
    in.R_wc = s.R_wc;

    fs::path d(s.sessionBuf);
    if (!fs::is_directory(d))
        return;

    auto mrp = parseMRP(d / "session_poses.mrp");
    if (mrp.empty())
        mrp = parseMRP(d / "session_ini_poses.mri");

    std::vector<fs::path> lazPaths;
    for (auto& e : fs::directory_iterator(d))
    {
        std::string n = e.path().filename().string();
        if (n.rfind("scan_lio_", 0) == 0 && e.path().extension() == ".laz")
            lazPaths.push_back(e.path());
    }
    std::sort(lazPaths.begin(), lazPaths.end());
    for (auto& lp : lazPaths)
    {
        RosExportInput::Chunk ch;
        ch.lazPath = lp.string();
        std::string key = lp.stem().string(); // "scan_lio_N"
        if (mrp.count(key))
        {
            ch.M = mrp.at(key);
            ch.hasM = true;
        }
        in.lidarChunks.push_back(std::move(ch));
    }
}

static void exportRos(AppState& s)
{
    if (s.rosBusy.load())
        return;

    // Gather the (owning) input on the UI thread, then run the heavy export on a
    // worker so the window keeps rendering. `in` and `opt` are owned by the thread.
    RosExportInput in;
    buildRosInput(s, in);
    RosExportOptions opt = s.ros;
    opt.outUri = s.rosOutBuf;
    opt.storageId = (s.rosStorageIdx == 1) ? "sqlite3" : "mcap";

    if (s.rosThread.joinable())
        s.rosThread.join();
    s.rosBusy = true;
    s.status = "Exporting ROS 2 bag... (see console)";
    s.rosThread = std::thread(
        [&s, in = std::move(in), opt]() mutable
        {
            std::string st;
            exportRos2Bag(in, opt, st);
            {
                std::lock_guard<std::mutex> lk(s.rosMtx);
                s.rosResult = std::move(st);
                s.rosResultReady = true;
            }
            s.rosBusy = false;
        });
}

static void drawScene(AppState& s)
{
    // ── trajectory path ───────────────────────────────────────────────────────
    if (s.showPath)
    {
        for (size_t i = 1; i < s.traj.poses.size(); i++)
        {
            auto& a = s.traj.poses[i - 1];
            auto& b = s.traj.poses[i];
            DrawLine3D(toRL(a.T.translation()), toRL(b.T.translation()), Color{ 100, 200, 255, 220 });
        }
    }

    // ── camera frustums ───────────────────────────────────────────────────────
    if (s.showFrustums && s.calibLoaded)
    {
        Eigen::Matrix3f R_wc = s.R_wc;
        Eigen::Vector3f C(s.E.tx, s.E.ty, s.E.tz);
        float fs = s.frustumScale;
        float ncx[4] = {
            (0.f - s.K.cx) / s.K.fx, (float(s.imgW) - s.K.cx) / s.K.fx, (float(s.imgW) - s.K.cx) / s.K.fx, (0.f - s.K.cx) / s.K.fx
        };
        float ncy[4] = {
            (0.f - s.K.cy) / s.K.fy, (0.f - s.K.cy) / s.K.fy, (float(s.imgH) - s.K.cy) / s.K.fy, (float(s.imgH) - s.K.cy) / s.K.fy
        };

        int64_t hlTs =
            (!s.imageTsNs.empty() && s.imgViewIdx >= 0 && s.imgViewIdx < (int)s.imageTsNs.size()) ? s.imageTsNs[s.imgViewIdx] : -1;

        for (int64_t ts : s.imageTsNs)
        {
            const TrajPose* pose = s.traj.nearest(ts);
            if (!pose)
                continue;

            Vector3 origin = toRL(pose->T * C);

            Vector3 w[4];
            for (int k = 0; k < 4; k++)
            {
                Eigen::Vector3f pl = R_wc * Eigen::Vector3f(ncx[k] * fs, ncy[k] * fs, fs) + C;
                w[k] = toRL(pose->T * pl);
            }

            bool hl = (ts == hlTs);
            Color fc = hl ? Color{ 255, 255, 50, 255 } : ORANGE;
            float sc = hl ? fs * 1.05f : fs;

            if (hl)
            {
                // filled quad highlight
                Vector3 w2[4];
                for (int k = 0; k < 4; k++)
                {
                    Eigen::Vector3f pl = R_wc * Eigen::Vector3f(ncx[k] * sc, ncy[k] * sc, sc) + C;
                    w2[k] = toRL(pose->T * pl);
                }
                DrawTriangle3D(w2[0], w2[1], w2[2], Color{ 255, 255, 50, 40 });
                DrawTriangle3D(w2[2], w2[3], w2[0], Color{ 255, 255, 50, 40 });
                DrawSphere(origin, fs * 0.04f, fc);
            }

            DrawLine3D(origin, w[0], fc);
            DrawLine3D(origin, w[1], fc);
            DrawLine3D(origin, w[2], fc);
            DrawLine3D(origin, w[3], fc);
            DrawLine3D(w[0], w[1], fc);
            DrawLine3D(w[1], w[2], fc);
            DrawLine3D(w[2], w[3], fc);
            DrawLine3D(w[3], w[0], fc);
        }
    }

    // ── GPU point cloud ───────────────────────────────────────────────────────
    if (s.cloud.count > 0 && s.shaderOk)
    {
        rlDrawRenderBatchActive();
        Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
        rlEnableShader(s.shader.id);
        rlSetUniformMatrix(s.locMVP, mvp);
        rlSetUniform(s.locPS, &s.pointSize, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(s.locCM, &s.colorMode, RL_SHADER_UNIFORM_INT, 1);
        rlSetUniform(s.locDecim, &s.drawDecim, RL_SHADER_UNIFORM_INT, 1);
        int sel = (s.isolateCamera && s.imgViewIdx >= 0 && s.imgViewIdx < (int)s.imageTsNs.size()) ? s.imgViewIdx : -1;
        rlSetUniform(s.locSel, &sel, RL_SHADER_UNIFORM_INT, 1);
        rlEnableVertexArray(s.cloud.vao);
        glDrawArrays(GL_POINTS, 0, s.cloud.count);
        rlDisableVertexArray();
        rlDisableShader();
    }
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    CliArgs args = parseArgs(argc, argv);
    static const char* kDesc = "View LIO trajectory, colorize and export point clouds";
    const std::vector<std::string> usage = { cliopt::MJS, cliopt::CAMERA_DIR, cliopt::CALIB };
    if (args.help)
    {
        printUsage("TrajectoryViewer", kDesc, usage);
        return 0;
    }
    if (!args.valid)
    {
        std::fprintf(stderr, "%s\n\n", args.error.c_str());
        printUsage("TrajectoryViewer", kDesc, usage, /*toStderr=*/true);
        return 1;
    }

    AppState s;
    // --mjs gives the session manifest; the session directory is its parent.
    std::string sessionDir;
    if (args.has("mjs"))
        sessionDir = fs::path(args.get("mjs")).parent_path().string();
    else if (!args.positional.empty())
        sessionDir = args.positional.front(); // back-compat
    if (!sessionDir.empty())
        strncpy(s.sessionBuf, sessionDir.c_str(), sizeof(s.sessionBuf) - 1);

    if (args.has("camera_dir"))
        strncpy(s.cameraBuf, args.get("camera_dir").c_str(), sizeof(s.cameraBuf) - 1);

    // --calib: calibration json (intrinsic + extrinsic). Fall back to any
    // positional ending in .json for backward compatibility.
    std::string calib = args.get("calib");
    if (calib.empty())
        for (const auto& p : args.positional)
            if (p.size() > 5 && p.substr(p.size() - 5) == ".json")
            {
                calib = p;
                break;
            }
    if (!calib.empty())
        strncpy(s.calibBuf, calib.c_str(), sizeof(s.calibBuf) - 1);

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1400, 900, ("Trajectory Viewer " HDMAPPING_VERSION_STRING));
    raylib_widgets::fitWindowToScreen();
    // panelW below is user-resizable but the 3D view still needs room.
    SetWindowMinSize(900, 500);
    SetTargetFPS(60);
    rlImGuiSetup(true);

    s.shader = LoadShaderFromMemory(kVS, kFS.c_str());
    s.shaderOk = s.shader.id > 0;
    if (s.shaderOk)
    {
        s.locMVP = rlGetLocationUniform(s.shader.id, "mvp");
        s.locPS = rlGetLocationUniform(s.shader.id, "pointSize");
        s.locCM = rlGetLocationUniform(s.shader.id, "colorMode");
        s.locDecim = rlGetLocationUniform(s.shader.id, "drawDecim");
        s.locSel = rlGetLocationUniform(s.shader.id, "selectedCamera");
    }
    glEnable(GL_PROGRAM_POINT_SIZE);

    // image viewer background loader thread
    s.imgViewThread = std::thread(
        [&s]()
        {
            int lastLoaded = -1;
            while (!s.imgViewStop.load())
            {
                int req = s.imgViewRequest.load();
                if (req != lastLoaded && req >= 0 && req < (int)s.imageTsNs.size())
                {
                    lastLoaded = req;
                    s.imgViewLoading = true;
                    int64_t ts = s.imageTsNs[req];
                    auto it = s.imagesFilenamesInTime.find(ts);
                    if (it != s.imagesFilenamesInTime.end())
                    {
                        cv::Mat img = cv::imread(it->second);
                        if (!img.empty())
                        {
                            cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
                            std::lock_guard<std::mutex> lk(s.imgViewMtx);
                            s.imgViewPending = std::move(img);
                            s.imgViewHasNew = true;
                        }
                    }
                    s.imgViewLoading = false;
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(8));
                }
            }
        });

    // auto-load if args given
    if (s.sessionBuf[0])
        loadSession(s);
    if (s.calibBuf[0])
        loadCalib(s);

    float panelW = 420.f;

    while (!WindowShouldClose())
    {
        bool imguiWants = ImGui::GetIO().WantCaptureMouse;
        s.orbit.update(!imguiWants);
        s.orbit.updateTransition(GetFrameTime());
        Camera3D cam = s.orbit.toRaylib();

        // pick up the ROS export result from the worker thread (if any)
        {
            std::lock_guard<std::mutex> lk(s.rosMtx);
            if (s.rosResultReady)
            {
                s.status = s.rosResult;
                s.rosResultReady = false;
            }
        }

        // Ctrl toggles point coloring: intensity (jet) <-> RGB
        if (!ImGui::GetIO().WantCaptureKeyboard)
        {
            if (IsKeyPressed(KEY_LEFT_CONTROL) || IsKeyPressed(KEY_RIGHT_CONTROL))
                s.colorMode = (s.colorMode == 1) ? 0 : 1;

            // Chord choices avoid colliding in MEANING with
            // multi_view_tls_registration_step_2's shortcuts (Ctrl+L there
            // is manual loop closure, Ctrl+E is the lio segments editor;
            // bare F there is the "camera Front" preset). Ctrl+O and bare
            // C/P are kept aligned with step2 (Ctrl+O = open/load session,
            // C = compass/ruler).
            bool ctrlDown = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
            bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            if (ctrlDown && shiftDown && IsKeyPressed(KEY_O))
                actionSelectCamera0Dir(s);
            else if (ctrlDown && IsKeyPressed(KEY_O))
                actionSelectLioResultDir(s);
            if (ctrlDown && shiftDown && IsKeyPressed(KEY_C))
                actionOpenCalibration(s);
            if (ctrlDown && IsKeyPressed(KEY_S))
                actionExportColoredPointCloud(s);

            if (!ctrlDown && IsKeyPressed(KEY_P))
                s.showPath = !s.showPath;
            if (!ctrlDown && IsKeyPressed(KEY_V))
                s.showFrustums = !s.showFrustums;
            if (!ctrlDown && IsKeyPressed(KEY_C))
                s.showCompassRuler = !s.showCompassRuler;

            if (shiftDown && IsKeyPressed(KEY_R))
                s.showCenterOfRotationWindow = true;
            if (!imguiWants && ctrlDown && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
                s.orbit.pickGroundPlaneTarget(GetMousePosition(), cam);
            if (!imguiWants && IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE))
                s.orbit.pickGroundPlaneTarget(GetMousePosition(), cam);

            if (IsKeyPressed(KEY_LEFT))
            {
                s.imgViewIdx = std::max(s.imgViewIdx - 1, 0);
                s.imgViewRequest.store(s.imgViewIdx);
            }
            if (IsKeyPressed(KEY_RIGHT))
            {
                s.imgViewIdx = std::min(s.imgViewIdx + 1, (int)s.imageTsNs.size());
                s.imgViewRequest.store(s.imgViewIdx);
            }
        }

        BeginDrawing();
        ClearBackground(Color{ 25, 25, 25, 255 });

        BeginMode3D(cam);
        drawScene(s);
        DrawGrid(20, 1.f);
        // axes
        DrawLine3D({ 0, 0, 0 }, { 2, 0, 0 }, RED);
        DrawLine3D({ 0, 0, 0 }, { 0, 2, 0 }, GREEN);
        DrawLine3D({ 0, 0, 0 }, { 0, 0, -2 }, BLUE);
        raylib_widgets::drawRotationCenterCross(s.orbit.target, s.orbit.distance * 0.05f, WHITE);
        EndMode3D();

        if (s.showCompassRuler)
        {
            Vector3 fwd = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, cam.up));
            Vector3 up = Vector3CrossProduct(right, fwd);
            raylib_widgets::drawCompassRuler(right, up, s.orbit.distance, LIGHTGRAY);
        }

        // ── upload image viewer texture if worker produced one ────────────────
        {
            cv::Mat toUpload;
            {
                std::lock_guard<std::mutex> lk(s.imgViewMtx);
                if (s.imgViewHasNew)
                {
                    std::swap(toUpload, s.imgViewPending);
                    s.imgViewHasNew = false;
                }
            }
            if (!toUpload.empty())
            {
                if (s.imgViewTexValid)
                    UnloadTexture(s.imgViewTex);
                Image ri = { toUpload.data, toUpload.cols, toUpload.rows, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8 };
                s.imgViewTex = LoadTextureFromImage(ri);
                s.imgViewTexValid = s.imgViewTex.id > 0;
            }
        }

        // ── ImGui panel ───────────────────────────────────────────────────────
        rlImGuiBegin();

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("File"))
            {
                if (ImGui::MenuItem("Select LIO Result Directory...", "Ctrl+O"))
                    actionSelectLioResultDir(s);
                if (ImGui::MenuItem("Select CAMERA_0 Directory...", "Ctrl+Shift+O"))
                    actionSelectCamera0Dir(s);
                ImGui::Separator();
                if (ImGui::MenuItem("Open Calibration...", "Ctrl+Shift+C"))
                    actionOpenCalibration(s);
                ImGui::Separator();
                if (ImGui::MenuItem("Export Colored Point Cloud...", "Ctrl+S"))
                    actionExportColoredPointCloud(s);
                ImGui::Separator();
                if (ImGui::MenuItem("Select ROS 2 Bag Output Directory..."))
                    actionSelectRosOutputDir(s);
                ImGui::Separator();
                if (ImGui::MenuItem("Select COLMAP Output Directory..."))
                    actionSelectColmapOutputDir(s);
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("View"))
            {
                ImGui::MenuItem("Show path", "P", &s.showPath);
                ImGui::MenuItem("Show frustums", "V", &s.showFrustums);
                ImGui::MenuItem("Show compass/ruler", "C", &s.showCompassRuler);
                if (ImGui::MenuItem("Center of rotation...", "Shift+R"))
                    s.showCenterOfRotationWindow = true;
                ImGui::Separator();
                ImGui::SetNextItemWidth(140.f);
                ImGui::SliderFloat("Frustum scale", &s.frustumScale, 0.05f, 5.f, "%.2f");
                ImGui::SetNextItemWidth(140.f);
                ImGui::SliderFloat("Point size", &s.pointSize, 1.f, 20.f, "%.1f");
                ImGui::SetNextItemWidth(140.f);
                ImGui::SliderInt("Draw decimation", &s.drawDecim, 1, 64);
                if (!s.imagesFilenamesInTime.empty())
                {
                    ImGui::Separator();
                    ImGui::TextDisabled("Point color:");
                    if (ImGui::MenuItem("Intensity", nullptr, s.colorMode == 0))
                        s.colorMode = 0;
                    if (ImGui::MenuItem("RGB (image)", "Ctrl", s.colorMode == 1))
                        s.colorMode = 1;
                    if (ImGui::MenuItem("Camera ID", nullptr, s.colorMode == 2))
                        s.colorMode = 2;
                    if (ImGui::MenuItem("In ROI", nullptr, s.colorMode == 3))
                        s.colorMode = 3;
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Help"))
            {
                ImGui::MenuItem("Shortcuts...", nullptr, &s.showHelp);
                ImGui::EndMenu();
            }

            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();

            constexpr float ImGuiNumberWidth = 120.0f;
            ImGui::SetNextItemWidth(ImGuiNumberWidth);

            ImGui::InputInt("Points render downsampling", &s.drawDecim, 2, 10);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("increase for better performance, decrease for rendering more points");
            ImGui::SameLine();

            // fps_avg = fps_avg * 0.7f + ImGui::GetIO().Framerate * 0.3f;  // exponential smoothing

            // double now = ImGui::GetTime();  // ImGui’s built-in timer (in seconds)

            // ImGui::Checkbox("dynamic", &dynamicSubsampling);
            // if (ImGui::IsItemHovered())
            //    ImGui::SetTooltip("automatically control subsampling vs FPS: increase bellow 10, decrease above 60");
            // if (dynamicSubsampling && (fps_avg < 15) && (now - lastAdjustTime > cooldownSeconds))
            //{
            //    app_state.viewer_decimate_point_cloud += 1;
            //    lastAdjustTime = now;
            //}
            // ImGui::SameLine();
            // ImGui::Text("(avg %.1f)", fps_avg);

            if (s.drawDecim < 1)
                s.drawDecim = 1;

            ImGui::SameLine();
            // GetFPS()/point-cloud draw-call/vertex count via raylib/ScanRenderer,
            // rather than ImGui's own Framerate tracker -- raylib doesn't
            // expose a general "draw calls" counter (rlgl's own internal one
            // only tracks its immediate-mode batch renderer, not custom
            // glDrawArrays calls like ScanRenderer's), so these are scan_renderer's
            // own per-frame counts of the calls/points it issued in draw().
            ImGui::Text("(%d FPS)", GetFPS());

            ImGui::EndMainMenuBar();
        }

        ImGuiIO& io = ImGui::GetIO();
        float menuBarH = ImGui::GetFrameHeight();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - panelW, menuBarH), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(panelW, io.DisplaySize.y - menuBarH), ImGuiCond_Always);
        ImGui::Begin("##panel", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
        panelW = ImGui::GetWindowWidth();

        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.f, 1.f), "Trajectory Viewer");
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Session", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::PushItemWidth(-1);
            ImGui::Text("LIO result directory:");
            ImGui::InputText("##sess", s.sessionBuf, sizeof(s.sessionBuf));
            ImGui::Text("CAMERA_0 directory (empty = auto):");
            ImGui::InputText("##cam", s.cameraBuf, sizeof(s.cameraBuf));
            if (ImGui::Button("Load session", ImVec2(-1, 0)))
                loadSession(s);
            if (!s.imagesFilenamesInTime.empty())
                ImGui::TextDisabled("%d images found", (int)s.imagesFilenamesInTime.size());
            ImGui::Separator();
            // Scoped narrower width: -1 (the block's default) gives this
            // trailing-label widget the full row and clips its label off
            // the right edge of the panel.
            ImGui::PopItemWidth();
            ImGui::PushItemWidth(-140.f);
            ImGui::InputInt("Load decimation", &s.cloudDecim);
            ImGui::PopItemWidth();
            ImGui::PushItemWidth(-1);
            s.cloudDecim = std::max(1, s.cloudDecim);
            ImGui::Checkbox("Multi-image coloring", &s.multiImgColoring);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("ON: all images per chunk, per-point assignment\nOFF: single image per chunk (midpoint)");
            ImGui::Text("Coloring strategy:");
            ImGui::RadioButton("Temporal", &s.colorStrategy, 0);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Match each point to the image nearest in time\n(searched outward up to 'Wiggle' frames, within 'Max time').");
            ImGui::SameLine();
            ImGui::RadioButton("Geometry", &s.colorStrategy, 1);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Match each point to the chunk image it projects into\nwith the smallest depth (closest camera).");
            if (s.colorStrategy == 0)
            {
                ImGui::PopItemWidth();
                ImGui::PushItemWidth(-140.f);
                ImGui::InputInt("Wiggle (frames)", &s.maxWiggle);
                s.maxWiggle = std::max(0, s.maxWiggle);
                ImGui::InputFloat("Max time (s)", &s.maxTemporalDist, 0.05f, 0.5f, "%.2f");
                s.maxTemporalDist = std::max(0.f, s.maxTemporalDist);
                ImGui::PopItemWidth();
                ImGui::PushItemWidth(-1);
            }
            if (ImGui::Button("Load cloud", ImVec2(-1, 0)))
                loadCloud(s);
            ImGui::PopItemWidth();
        }

        if (ImGui::CollapsingHeader("Calibration", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::PushItemWidth(-1);
            ImGui::Text("Calibration JSON:");
            ImGui::InputText("##cal", s.calibBuf, sizeof(s.calibBuf));
            if (ImGui::Button("Load calibration", ImVec2(-1, 0)))
                loadCalib(s);
            if (s.calibLoaded)
            {
                ImGui::Text("fx=%.0f fy=%.0f", s.K.fx, s.K.fy);
                ImGui::Text("cx=%.0f cy=%.0f", s.K.cx, s.K.cy);

                ImGui::Separator();
                if (ImGui::Checkbox("Region of interest", &s.roi.enabled))
                {
                    // first enable with an empty ROI: default to the full image
                    if (s.roi.enabled && (s.roi.w <= 0 || s.roi.h <= 0))
                    {
                        s.roi.x = 0;
                        s.roi.y = 0;
                        s.roi.w = s.imgW;
                        s.roi.h = s.imgH;
                    }
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Only points projecting inside the ROI get colored.\nDrawn on the image preview.");
                if (s.roi.enabled)
                {
                    ImGui::PopItemWidth();
                    ImGui::PushItemWidth(-140.f);
                    ImGui::InputInt("ROI x", &s.roi.x);
                    ImGui::InputInt("ROI y", &s.roi.y);
                    ImGui::InputInt("ROI w", &s.roi.w);
                    ImGui::InputInt("ROI h", &s.roi.h);
                    ImGui::PopItemWidth();
                    ImGui::PushItemWidth(-1);
                }
            }
            ImGui::PopItemWidth();
        }

        if (ImGui::CollapsingHeader("Image Preview", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (s.imageTsNs.empty())
            {
                ImGui::TextDisabled("Load session first");
            }
            else
            {
                int nImgs = (int)s.imageTsNs.size();
                ImGui::PushItemWidth(-1);
                bool moved = ImGui::SliderInt("##imgidx", &s.imgViewIdx, 0, nImgs - 1);
                ImGui::PopItemWidth();
                ImGui::SameLine(0, 4);
                ImGui::TextDisabled("%d/%d", s.imgViewIdx + 1, nImgs);
                if (moved)
                {
                    s.imgViewIdx = std::clamp(s.imgViewIdx, 0, nImgs - 1);
                    s.imgViewRequest.store(s.imgViewIdx);
                }
                ImGui::TextDisabled("ts: %lld", (long long)s.imageTsNs[s.imgViewIdx]);
                ImGui::Checkbox("Only this camera's points", &s.isolateCamera);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Render only points colored by the selected image.\nNeeds 'Color by image (RGB)' enabled.");
                if (s.imgViewLoading.load())
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Loading...");
                else if (s.imgViewTexValid)
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "%dx%d", s.imgViewTex.width, s.imgViewTex.height);
            }
        }

        if (ImGui::CollapsingHeader("Export", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::PushItemWidth(-1);
            ImGui::Text("Output file (.laz / .las):");
            ImGui::InputText("##out", s.exportBuf, sizeof(s.exportBuf));
            if (ImGui::Button("Export colored LAZ", ImVec2(-1, 0)))
                exportLAZ(s);
            if (!s.exportCloud.empty())
                ImGui::TextDisabled("%d pts ready to export", (int)s.exportCloud.size());
            ImGui::PopItemWidth();
        }

        if (ImGui::CollapsingHeader("ROS 2 Export"))
        {
#ifdef CALIB_ENABLE_ROS_EXPORT
            ImGui::PushItemWidth(-1);
            ImGui::Text("Output bag directory:");
            ImGui::InputText("##rosout", s.rosOutBuf, sizeof(s.rosOutBuf));
            // Scoped narrower width -- see the "Load decimation" comment above.
            ImGui::PopItemWidth();
            ImGui::PushItemWidth(-140.f);
            ImGui::Combo("Storage", &s.rosStorageIdx, "mcap\0sqlite3\0");
            ImGui::PopItemWidth();
            ImGui::PushItemWidth(-1);

            ImGui::Separator();
            ImGui::Checkbox("TF + static TF", &s.ros.exportTf);
            ImGui::Checkbox("Camera", &s.ros.exportCamera);
            if (s.ros.exportCamera)
            {
                ImGui::Indent();
                ImGui::Checkbox("Compressed (jpeg)", &s.ros.compressCamera);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("ON: CompressedImage (jpeg)\nOFF: raw Image bgr8");
                ImGui::Checkbox("Undistort (rectify)", &s.ros.undistortCamera);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Rectify to pinhole so RViz overlays line up\n(CameraInfo published with zero distortion).");
                ImGui::Unindent();
            }
            ImGui::Checkbox("LiDAR undistorted (map frame)", &s.ros.exportLidarUndistorted);
            ImGui::Checkbox("LiDAR raw (sensor frame)", &s.ros.exportLidarRaw);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Re-projects points into the lidar frame per-point\nusing the trajectory (needs poses loaded).");

            ImGui::Separator();
            // Scoped narrower width -- see the "Load decimation" comment above.
            ImGui::PopItemWidth();
            ImGui::PushItemWidth(-140.f);
            ImGui::InputDouble("Aggregation (s)", &s.ros.aggregationSec, 0.01, 0.1, "%.3f");
            s.ros.aggregationSec = std::max(0.001, s.ros.aggregationSec);
            ImGui::InputInt("LiDAR decimation", &s.ros.lidarDecim);
            s.ros.lidarDecim = std::max(1, s.ros.lidarDecim);
            ImGui::PopItemWidth();
            ImGui::PushItemWidth(-1);

            if (s.rosBusy.load())
            {
                ImGui::BeginDisabled();
                ImGui::Button("Exporting...", ImVec2(-1, 0));
                ImGui::EndDisabled();
            }
            else if (ImGui::Button("Export ROS 2 bag", ImVec2(-1, 0)))
            {
                exportRos(s);
            }
            ImGui::PopItemWidth();
#else
            ImGui::TextDisabled("Not available in this build");
            ImGui::TextDisabled("(rebuild with -DCALIB_ENABLE_ROS_EXPORT=ON)");
#endif
        }

        if (ImGui::CollapsingHeader("COLMAP Export"))
        {
            ImGui::PushItemWidth(-1);
            ImGui::Text("Output project dir:");
            ImGui::InputText("##colmapout", s.colmapBuf, sizeof(s.colmapBuf));
            ImGui::Checkbox("Copy images into project", &s.colmapCopyImages);
            // Scoped narrower width -- see the "Load decimation" comment above.
            ImGui::PopItemWidth();
            ImGui::PushItemWidth(-140.f);
            ImGui::InputInt("Point decimation", &s.colmapPtDecim);
            ImGui::PopItemWidth();
            ImGui::PushItemWidth(-1);
            s.colmapPtDecim = std::max(1, s.colmapPtDecim);
            if (ImGui::Button("Export COLMAP model", ImVec2(-1, 0)))
                exportColmap(s);
            ImGui::TextDisabled("Writes sparse/{cameras,images,points3D}.txt");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Needs calibration + a loaded cloud (for points3D).\n"
                    "Point COLMAP image_path at the images dir.");
            ImGui::PopItemWidth();
        }

        if (!s.status.empty())
        {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "%s", s.status.c_str());
        }

        ImGui::Separator();
        ImGui::TextDisabled("LMB: orbit  RMB: pan  Scroll: zoom");

        ImGui::End();

        raylib_widgets::showCenterOfRotationWindow(s.showCenterOfRotationWindow, s.orbit);

        // ── shortcuts help window ───────────────────────────────────────────────
        if (s.showHelp)
        {
            ImGui::SetNextWindowSize(ImVec2(420, 320), ImGuiCond_FirstUseEver);
            if (ImGui::Begin("Shortcuts", &s.showHelp))
                raylib_widgets::ShowShortcutsTable(appShortcuts);
            ImGui::End();
        }

        // ── floating image viewer window ──────────────────────────────────────
        if (s.imgViewTexValid)
        {
            ImGui::SetNextWindowPos(ImVec2(8, 8), ImGuiCond_Once);
            ImGui::SetNextWindowSize(ImVec2(640, 480), ImGuiCond_Once);
            ImGui::Begin("Image##viewer", nullptr, ImGuiWindowFlags_NoScrollbar);
            ImVec2 avail = ImGui::GetContentRegionAvail();
            float aspect = (float)s.imgViewTex.height / (float)s.imgViewTex.width;
            int dispW = (int)avail.x;
            int dispH = (int)(avail.x * aspect);
            if (dispH > (int)avail.y)
            {
                dispH = (int)avail.y;
                dispW = (int)(avail.y / aspect);
            }
            ImVec2 imgPos = ImGui::GetCursorScreenPos();
            rlImGuiImageSize(&s.imgViewTex, dispW, dispH);
            // overlay the ROI, mapping full-res image pixels to the displayed rect
            if (s.roi.enabled && s.imgViewTex.width > 0 && s.imgViewTex.height > 0)
            {
                float sx = (float)dispW / s.imgViewTex.width;
                float sy = (float)dispH / s.imgViewTex.height;
                ImVec2 a(imgPos.x + s.roi.x * sx, imgPos.y + s.roi.y * sy);
                ImVec2 b(imgPos.x + (s.roi.x + s.roi.w) * sx, imgPos.y + (s.roi.y + s.roi.h) * sy);
                ImGui::GetWindowDrawList()->AddRect(
                    a,
                    b,
                    IM_COL32(0, 255, 0, 255),
                    /*rounding=*/0.f,
                    /*thickness=*/2.f);
            }
            ImGui::End();
        }

        rlImGuiEnd();
        EndDrawing();
    }

    s.imgViewStop = true;
    s.imgViewThread.join();
    if (s.rosThread.joinable())
        s.rosThread.join();
    if (s.imgViewTexValid)
        UnloadTexture(s.imgViewTex);

    s.cloud.unload();
    if (s.shaderOk)
        UnloadShader(s.shader);
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
