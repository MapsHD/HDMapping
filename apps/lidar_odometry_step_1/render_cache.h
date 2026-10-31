#pragma once

#include "lidar_odometry_utils.h"
#include <Core/raylib_render.hpp>
#include <Core/raylib_utils.hpp>
#include <chrono>
#include <memory>

namespace step1_render
{
    inline PointVertex vertex(const Eigen::Vector3d& p, const Eigen::Vector3f& color)
    {
        return { static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()), color.x(), color.y(), color.z() };
    }

    inline void line(std::vector<PointVertex>& vertices, const Eigen::Vector3d& a, const Eigen::Vector3d& b, Eigen::Vector3f color)
    {
        // Match rlColor3f's byte colors for lines; point colors retain their original float precision.
        for (int i = 0; i < 3; ++i)
            color[i] = static_cast<unsigned char>(color[i] * 255) / 255.f;
        vertices.push_back(vertex(a, color));
        vertices.push_back(vertex(b, color));
    }

    inline Eigen::Vector3f shaded(Eigen::Vector3f color, float intensity, const IntensityRange& range, bool enabled)
    {
        return enabled ? color * (0.35f + 0.65f * std::sqrt(range.normalize(intensity))) : color;
    }

    struct PointCache
    {
        std::vector<raylib_widgets::PointBufferPart> parts;
        uint64_t revision = 0;
        int stride = 0;
        bool shading = false;

        void draw(
            const std::vector<Point3Di>& source,
            uint64_t version,
            int decimation,
            bool shade,
            const Eigen::Vector3f& color,
            float size,
            const Eigen::Affine3d& transform = Eigen::Affine3d::Identity())
        {
            decimation = std::max(1, decimation);
            if (revision != version || stride != decimation || shading != shade)
            {
                const auto range = intensityRange(source, decimation);
                std::vector<PointVertex> vertices;
                vertices.reserve((source.size() + decimation - 1) / decimation);
                for (size_t i = 0; i < source.size(); i += decimation)
                    vertices.push_back(vertex(source[i].point, shaded(color, source[i].intensity, range, shade)));
                uploadPointVertices(parts, vertices);
                revision = version;
                stride = decimation;
                shading = shade;
            }
            drawPointVertices(parts, size, transform);
        }
    };

    enum Display : unsigned
    {
        Points = 1,
        Axes = 2,
        Predictions = 4,
        PredictionAxes = 8
    };

    struct TrajectoryCache
    {
        ScanRenderer::PointsGPU points;
        std::vector<raylib_widgets::PointBufferPart> axes, predictions, prediction_axes;
        std::weak_ptr<const std::vector<WorkerData>> preview;
        unsigned built = 0;
        bool was_running = false;

        void draw(
            ScanRenderer& renderer,
            const std::vector<WorkerData>& workers,
            const std::shared_ptr<const std::vector<WorkerData>>& current,
            bool running,
            bool& dirty,
            unsigned options)
        {
            if (!options)
                return;
            if (dirty || current != preview.lock() || running != was_running)
            {
                built = 0;
                preview = current;
                was_running = running;
                dirty = false;
            }
            const unsigned missing = options & ~built;
            if (missing & Points)
            {
                size_t count = 0;
                for (const auto& wd : workers)
                    count += wd.intermediate_trajectory.size();
                std::vector<Eigen::Vector3d> positions;
                positions.reserve(count);
                for (const auto& wd : workers)
                    for (const auto& pose : wd.intermediate_trajectory)
                        positions.emplace_back(pose.translation());
                renderer.uploadPoints(points, positions);
            }
            if (missing & (Axes | PredictionAxes))
            {
                std::vector<PointVertex> actual, predicted;
                for (const auto& wd : workers)
                    for (size_t i = 0; i < wd.intermediate_trajectory.size(); ++i)
                    {
                        const auto& pose = wd.intermediate_trajectory[i];
                        const Eigen::Vector3d origin = pose.translation();
                        for (int axis = 0; axis < 3; ++axis)
                        {
                            if (missing & Axes)
                                line(actual, origin, origin + pose.linear().col(axis) * 0.1, Eigen::Vector3f::Unit(axis));
                            if ((missing & PredictionAxes) && i < wd.intermediate_trajectory_prediction.size())
                                line(
                                    predicted,
                                    origin,
                                    origin + wd.intermediate_trajectory_prediction[i].linear().col(axis) * 0.05,
                                    Eigen::Vector3f::Unit(axis));
                        }
                    }
                if (missing & Axes)
                    uploadPointVertices(axes, actual);
                if (missing & PredictionAxes)
                    uploadPointVertices(prediction_axes, predicted);
            }
            if (missing & Predictions)
            {
                std::vector<PointVertex> vertices;
                for (const auto& wd : workers)
                    if (!wd.intermediate_trajectory.empty() && wd.imu_prediction_vector.norm() >= 1e-6)
                    {
                        const Eigen::Vector3d origin = wd.intermediate_trajectory.front().translation();
                        line(vertices, origin, origin + wd.imu_prediction_vector, Eigen::Vector3f(1, 0, 1));
                    }
                uploadPointVertices(predictions, vertices);
            }
            built |= missing;
            if (options & Axes)
                drawLineVertices(axes);
            if (options & Predictions)
            {
                rlDrawRenderBatchActive();
                rlDisableDepthTest();
                drawLineVertices(predictions);
                rlEnableDepthTest();
            }
            if (options & PredictionAxes)
                drawLineVertices(prediction_axes);
            if (options & Points)
                renderer.drawPoints(points, Color{ 0, 255, 255, 255 }, 3.f);
        }

        void clear(ScanRenderer& renderer)
        {
            renderer.unloadPoints(points);
            unloadPointVertices(axes);
            unloadPointVertices(predictions);
            unloadPointVertices(prediction_axes);
            preview.reset();
            built = 0;
        }
    };

    struct BucketCache
    {
        // A display projection, not a copy of the optimizer's hash map or inverse covariance state.
        struct Sample
        {
            Eigen::Vector3d position;
            float intensity;
            char hits;
            bool marked;
        };
        std::vector<Sample> samples;
        std::vector<Eigen::Vector3d> normal_data;
        std::vector<Eigen::Matrix3d> covariance_data;
        std::vector<raylib_widgets::PointBufferPart> points, normals;
        std::vector<Eigen::Affine3d> ellipses;
        uint64_t revision = 0;
        unsigned details = 0, built = 0;
        bool valid = false, shading = false, filtered = false;
        std::chrono::steady_clock::time_point last_attempt{};
        IntensityRange range;

        // flags: points=1, normals=2, covariance=4. Versions are read only under the map mutex.
        void prepare(
            const NDTBucketMapType& source,
            std::mutex& mutex,
            const uint64_t& version,
            unsigned flags,
            bool running,
            bool shade,
            bool filter,
            const Eigen::Vector3f& color,
            const Eigen::Vector3f& marked_color)
        {
            if (!flags)
                return;
            const auto now = std::chrono::steady_clock::now();
            bool changed = false;
            if (!valid || details != (flags & 6) || !running || now - last_attempt >= std::chrono::milliseconds(100))
            {
                last_attempt = now;
                std::unique_lock lock(mutex, std::try_to_lock);
                if (lock && (!valid || revision != version || details != (flags & 6)))
                {
                    samples.clear();
                    samples.reserve(source.size());
                    normal_data.clear();
                    covariance_data.clear();
                    if (flags & 2)
                        normal_data.reserve(source.size());
                    if (flags & 4)
                        covariance_data.reserve(source.size());
                    for (const auto& entry : source)
                    {
                        const auto& b = entry.second;
                        samples.push_back({ b.mean, b.intensity, b.number_of_hits, b.number_of_points == uint64_t(-1) });
                        if (flags & 2)
                            normal_data.push_back(b.normal_vector);
                        if (flags & 4)
                            covariance_data.push_back(b.cov);
                    }
                    revision = version;
                    details = flags & 6;
                    valid = changed = true;
                    built = 0;
                }
            } // no color calculations, covariance factorization or GPU work under the map mutex
            if (changed)
            {
                range = {};
                bool first = true;
                for (const auto& p : samples)
                    if (!p.marked)
                    {
                        range.min = first ? p.intensity : std::min(range.min, p.intensity);
                        range.max = first ? p.intensity : std::max(range.max, p.intensity);
                        first = false;
                    }
            }
            if ((flags & 1) && (!(built & 1) || shading != shade || filtered != filter))
            {
                std::vector<PointVertex> vertices;
                vertices.reserve(samples.size());
                for (const auto& p : samples)
                    if (!filter || p.hits < 20)
                        vertices.push_back(vertex(p.position, p.marked ? marked_color : shaded(color, p.intensity, range, shade)));
                uploadPointVertices(points, vertices);
                shading = shade;
                filtered = filter;
                built |= 1;
            }
            if ((flags & details & 2) && !(built & 2))
            {
                std::vector<PointVertex> vertices;
                vertices.reserve(samples.size() * 2);
                for (size_t i = 0; i < samples.size(); ++i)
                    line(vertices, samples[i].position, samples[i].position + normal_data[i], normal_data[i].cwiseAbs().cast<float>());
                uploadPointVertices(normals, vertices);
                built |= 2;
            }
            if ((flags & details & 4) && !(built & 4))
            {
                ellipses.clear();
                ellipses.reserve(samples.size());
                for (size_t i = 0; i < samples.size(); ++i)
                {
                    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
                    transform.linear() = Eigen::LLT<Eigen::Matrix3d>(covariance_data[i]).matrixL().toDenseMatrix() * 3;
                    transform.translation() = samples[i].position;
                    ellipses.push_back(transform);
                }
                built |= 4;
            }
        }

        void drawCovariances(const Eigen::Vector3f& color) const
        {
            if (built & 4)
                for (const auto& transform : ellipses)
                    draw_ellipse(transform, color);
        }

        void clear()
        {
            unloadPointVertices(points);
            unloadPointVertices(normals);
            samples.clear();
            normal_data.clear();
            covariance_data.clear();
            ellipses.clear();
            built = details = 0;
            valid = false;
        }
    };
} // namespace step1_render
