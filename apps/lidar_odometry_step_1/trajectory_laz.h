#pragma once

#include <Core/transformations.h>
#include <laszip/laszip_api.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

namespace step1_export
{
    //! One world-space point per stored optimized pose; named LAS Extra Bytes carry pose scalars.
    inline bool trajectoryLaz(const std::filesystem::path& path, const std::vector<WorkerData>& workers, std::string& error)
    {
        const auto temporary =
            std::filesystem::path(path.string() + ".tmp." + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        try
        {
            uint64_t count = 0;
            Eigen::Vector3d minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
            Eigen::Vector3d maximum = -minimum;
            for (const auto& worker : workers)
                for (const auto& pose : worker.intermediate_trajectory)
                {
                    if (!pose.matrix().allFinite())
                        throw std::runtime_error("Non-finite trajectory pose");
                    minimum = minimum.cwiseMin(pose.translation());
                    maximum = maximum.cwiseMax(pose.translation());
                    ++count;
                }
            if (!count)
                throw std::runtime_error("No trajectory poses to export");
            const Eigen::Vector3d offset = minimum * 0.5 + maximum * 0.5;

            laszip_POINTER raw = nullptr;
            if (laszip_create(&raw))
                throw std::runtime_error("Cannot create trajectory LAZ writer");
            bool opened = false;
            auto destroy = [&opened](void* handle)
            {
                if (opened)
                    laszip_close_writer(handle);
                laszip_destroy(handle);
            };
            std::unique_ptr<void, decltype(destroy)> writer(raw, destroy);
            auto check = [&](laszip_I32 result, const char* operation)
            {
                if (result)
                {
                    laszip_CHAR* message = nullptr;
                    laszip_get_error(raw, &message);
                    throw std::runtime_error(std::string(operation) + (message ? ": " + std::string(message) : ""));
                }
            };
            auto put = [](laszip_U8* destination, auto value)
            {
                std::array<laszip_U8, sizeof(value)> bytes;
                std::memcpy(bytes.data(), &value, sizeof(value));
                if constexpr (std::endian::native == std::endian::big)
                    std::reverse(bytes.begin(), bytes.end());
                std::memcpy(destination, bytes.data(), bytes.size());
            };
            struct Field
            {
                const char* name;
                const char* description;
                laszip_U32 type;
                size_t size;
            };
            const Field fields[] = {
                { "pose_index", "Zero-based global pose index", 6, 8 },      { "worker_index", "Zero-based source worker index", 4, 4 },
                { "pose_in_worker", "Zero-based pose within worker", 4, 4 }, { "time_sensor_s", "Sensor clock [s]; NaN if absent", 9, 8 },
                { "time_unix_s", "POSIX UTC [s]; NaN if absent", 9, 8 },     { "time_elapsed_s", "Since first sensor time [s]", 9, 8 },
                { "pose_roll_deg", "Optimized RxRyRz X angle [deg]", 9, 8 }, { "pose_pitch_deg", "Optimized RxRyRz Y angle [deg]", 9, 8 },
                { "pose_yaw_deg", "Optimized RxRyRz Z angle [deg]", 9, 8 },  { "distance_m", "Cumulative pose-order distance m", 9, 8 },
                { "speed_mps", "Backward segment speed [m/s]", 9, 8 },       { "segment_valid", "1: speed valid; 0: unavailable", 0, 1 },
            };
            laszip_header* header = nullptr;
            check(laszip_get_header_pointer(raw, &header), "Get trajectory header");
            header->version_major = 1;
            header->version_minor = 4;
            header->header_size = header->offset_to_point_data = 375;
            header->global_encoding = 0;
            header->point_data_format = 0; // Sensor/Unix times are named Extra Bytes, not LAS GPS time.
            header->point_data_record_length = 20;
            header->extended_number_of_point_records = count;
            header->extended_number_of_points_by_return[0] = count;
            header->number_of_point_records = count <= UINT32_MAX ? static_cast<laszip_U32>(count) : 0;
            header->number_of_points_by_return[0] = header->number_of_point_records;
            std::snprintf(header->generating_software, sizeof(header->generating_software), "HDMapping Step 1 trajectory");
            double* scales[] = { &header->x_scale_factor, &header->y_scale_factor, &header->z_scale_factor };
            double* offsets[] = { &header->x_offset, &header->y_offset, &header->z_offset };
            for (int axis = 0; axis < 3; ++axis)
            {
                *offsets[axis] = offset[axis];
                *scales[axis] = std::max(0.0001, (maximum[axis] - minimum[axis]) / (2.0 * (INT32_MAX - 1)));
                if (!std::isfinite(*scales[axis]))
                    throw std::runtime_error("Trajectory extent exceeds LAS coordinate range");
            }
            for (const auto& field : fields)
            {
                check(laszip_add_attribute(raw, field.type, field.name, field.description, 1, 0), "Add trajectory scalar");
                header->point_data_record_length += static_cast<laszip_U16>(field.size);
            }
            const double missing = std::numeric_limits<double>::quiet_NaN();
            // LAS Extra Bytes descriptor: options byte 3; double no_data at byte 40.
            for (laszip_U32 i = 0; i < header->number_of_variable_length_records; ++i)
                if (header->vlrs[i].record_id == 4 && std::strncmp(header->vlrs[i].user_id, "LASF_Spec", 9) == 0)
                    for (size_t j = 0; j < std::size(fields); ++j)
                    {
                        auto* descriptor = header->vlrs[i].data + j * 192;
                        // Values are already in their declared units; preserve integer index types.
                        descriptor[3] &= static_cast<laszip_U8>(~(8 | 16));
                        if (fields[j].type == 9)
                        {
                            descriptor[3] |= 1;
                            put(descriptor + 40, missing);
                        }
                    }
            const std::string metadata =
                R"({"kind":"trajectory","schema_version":1,"coordinate_frame":"Step 1 world; same as session poses","pose_source":"all stored optimized poses, including trailing workers","rotation":"R=Rx(roll)*Ry(pitch)*Rz(yaw); degrees; yaw is not compass heading","time_sensor_s":"original sensor seconds, not LAS GPS time","time_unix_s":"original POSIX seconds; NaN when absent","time_elapsed_s":"relative to first available sensor timestamp","distance_m":"Euclidean distance accumulated in stored pose order","speed_mps":"distance from previous stored pose divided by positive sensor dt; NaN otherwise","segment_valid":"0 for first pose, absent/non-increasing time or non-finite speed; otherwise 1","missing":"floating-point NaN, declared in Extra Bytes no_data"})";
            check(
                laszip_add_vlr(
                    raw,
                    "HDMapping",
                    1,
                    static_cast<laszip_U16>(metadata.size()),
                    "Step 1 trajectory schema",
                    reinterpret_cast<const laszip_U8*>(metadata.data())),
                "Add trajectory metadata");
            check(laszip_open_writer(raw, temporary.string().c_str(), 1), "Open trajectory LAZ");
            opened = true;
            laszip_point* point = nullptr;
            check(laszip_get_point_pointer(raw, &point), "Get trajectory point");
            size_t extraSize = 0;
            for (const auto& field : fields)
                extraSize += field.size;
            if (point->num_extra_bytes != static_cast<int>(extraSize))
                throw std::runtime_error("Trajectory Extra Bytes layout mismatch");
            uint64_t index = 0;
            std::optional<double> firstTime;
            double previousTime = missing, distance = 0;
            Eigen::Vector3d previousPosition = Eigen::Vector3d::Zero();
            for (size_t workerIndex = 0; workerIndex < workers.size(); ++workerIndex)
            {
                const auto& worker = workers[workerIndex];
                if (workerIndex > UINT32_MAX || worker.intermediate_trajectory.size() > UINT32_MAX)
                    throw std::runtime_error("Worker/pose index exceeds exported scalar range");
                for (size_t j = 0; j < worker.intermediate_trajectory.size(); ++j, ++index)
                {
                    const auto& pose = worker.intermediate_trajectory[j];
                    const auto position = pose.translation().eval();
                    double time =
                        j < worker.intermediate_trajectory_timestamps.size() ? worker.intermediate_trajectory_timestamps[j].first : missing;
                    double unixTime = j < worker.intermediate_trajectory_timestamps.size()
                        ? worker.intermediate_trajectory_timestamps[j].second
                        : missing;
                    if (!std::isfinite(time))
                        time = missing;
                    if (!std::isfinite(unixTime) || unixTime <= 0)
                        unixTime = missing;
                    if (!firstTime && std::isfinite(time))
                        firstTime = time;
                    double elapsed = firstTime ? time - *firstTime : missing;
                    if (!std::isfinite(elapsed))
                        elapsed = missing;
                    const double step = index ? (position - previousPosition).norm() : 0;
                    distance += step;
                    const double dt = time - previousTime;
                    double speed = index && std::isfinite(dt) && dt > 0 ? step / dt : missing;
                    if (!std::isfinite(speed))
                        speed = missing;
                    if (!std::isfinite(distance))
                        throw std::runtime_error("Trajectory distance overflow");
                    const auto angles = pose_tait_bryan_from_affine_matrix(pose);
                    size_t cursor = 0;
                    auto append = [&](auto value)
                    {
                        put(point->extra_bytes + cursor, value);
                        cursor += sizeof(value);
                    };
                    append(index);
                    append(static_cast<uint32_t>(workerIndex));
                    append(static_cast<uint32_t>(j));
                    append(time);
                    append(unixTime);
                    append(elapsed);
                    append(rad2deg(angles.om));
                    append(rad2deg(angles.fi));
                    append(rad2deg(angles.ka));
                    append(distance);
                    append(speed);
                    append(static_cast<uint8_t>(std::isfinite(speed)));
                    point->return_number = point->number_of_returns = 1;
                    laszip_F64 coordinates[] = { position.x(), position.y(), position.z() };
                    check(laszip_set_coordinates(raw, coordinates), "Set trajectory coordinates");
                    check(laszip_write_point(raw), "Write trajectory pose");
                    check(laszip_update_inventory(raw), "Update trajectory inventory");
                    previousTime = time;
                    previousPosition = position;
                }
            }
            check(laszip_close_writer(raw), "Close trajectory LAZ");
            opened = false;
            writer.reset();
#ifdef _WIN32
            if (!MoveFileExW(temporary.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING))
                throw std::runtime_error("Cannot replace trajectory LAZ");
#else
            std::filesystem::rename(temporary, path);
#endif
            error.clear();
            return true;
        } catch (const std::exception& failure)
        {
            error = failure.what();
            std::error_code ignored;
            std::filesystem::remove(temporary, ignored);
            return false;
        }
    }
} // namespace step1_export
