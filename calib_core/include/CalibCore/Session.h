#pragma once
#include <CalibCore/Trajectory.h>
#include <Eigen/Geometry>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace calib {

//! One session's files, read from its *.mjs manifest rather than by scanning
//! the directory -- a directory can hold more than one session, each with its
//! own poses file and chunk list.
struct SessionFiles
{
    std::filesystem::path mjs;
    //! Chunk stem (e.g. "scan_lio_0") -> world pose; poses_file_name, else initial_poses_file_name.
    std::map<std::string, Eigen::Affine3f> poses;
    bool hasPoses = false;
    //! laz_file_names, in manifest order.
    std::vector<std::filesystem::path> lazPaths;
};

//! Reads a poses file (session_poses.mrp / session_ini_poses.mri).
//! @param mrpPath file to read
//! @return chunk stem (e.g. "scan_lio_0") -> pose; empty when the file can't be read
std::map<std::string, Eigen::Affine3f> parseMRP(const std::filesystem::path& mrpPath);

//! The session manifest a path names: a *.mjs file as is; a directory resolves
//! to its session.mjs (what lidar_odometry_step_1 writes), else its only *.mjs.
//! @param p file or directory
//! @param err receives the reason when there is no single answer
//! @return the manifest, or an empty path
std::filesystem::path resolveMjs(const std::filesystem::path& p, std::string& err);

//! Reads the manifest's poses and chunk list. Stored paths resolve as in
//! Session::load (see HDMapping/SessionPaths.h).
//! @param mjs manifest to read
//! @param out overwritten with the session's files
//! @param err receives the reason on failure
//! @return false when the manifest can't be read or parsed
bool readSessionFiles(const std::filesystem::path& mjs, SessionFiles& out, std::string& err);

//! A "scan_lio_N" chunk's trajectory: trajectory_lio_N.csv next to its laz.
std::filesystem::path chunkTrajectoryCsv(const std::filesystem::path& lazPath);

//! Appends every chunk's trajectory CSV to `traj`, each with its chunk's pose
//! applied, then sorts it. Chunks without a CSV are skipped.
void loadSessionTrajectory(const SessionFiles& sf, Trajectory& traj);

}  // namespace calib
