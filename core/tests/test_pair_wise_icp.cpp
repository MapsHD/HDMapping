#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <Core/pair_wise_iterative_closest_point.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace
{
    /**
     * @brief Portable pseudo-random generator (splitmix64), so the scene is the same on every platform and
     * standard library (std::uniform_real_distribution is implementation-defined).
     */
    class SplitMix64
    {
    public:
        explicit SplitMix64(uint64_t seed)
            : state(seed)
        {
        }

        /**
         * @brief Next uniform value.
         * @return Value in [lo, hi).
         */
        double uniform(double lo, double hi)
        {
            uint64_t z = (state += 0x9E3779B97F4A7C15ull);
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
            z = z ^ (z >> 31);
            return lo + (hi - lo) * static_cast<double>(z >> 11) * (1.0 / 9007199254740992.0);
        }

    private:
        uint64_t state;
    };

    /**
     * @brief Synthetic room scan: floor, ceiling, four walls and three boxes, sampled on a jittered 0.1 m grid.
     * The boxes break the room's symmetry so all six pose parameters are constrained.
     * @return Points in the target (world) frame.
     */
    std::vector<Eigen::Vector3d> make_room()
    {
        SplitMix64 rng(1);
        std::vector<Eigen::Vector3d> points;
        const double step = 0.1;
        const double x0 = 2.0, x1 = 14.0, y0 = 3.0, y1 = 11.0, z0 = 0.0, z1 = 3.0;
        const auto jitter = [&]()
        {
            return rng.uniform(-0.03, 0.03);
        };
        // Jitter drawn in x, y, z order as separate statements: the evaluation order of function arguments is
        // unspecified (clang: left to right; GCC, MSVC: right to left), so jitter() calls inside one argument list
        // would build a different room on each compiler.
        const auto add_jittered = [&](double x, double y, double z, bool jitter_x, bool jitter_y, bool jitter_z)
        {
            const double dx = jitter_x ? jitter() : 0.0;
            const double dy = jitter_y ? jitter() : 0.0;
            const double dz = jitter_z ? jitter() : 0.0;
            points.emplace_back(x + dx, y + dy, z + dz);
        };

        for (double x = x0; x <= x1; x += step)
        {
            for (double y = y0; y <= y1; y += step)
            {
                add_jittered(x, y, z0, true, true, false);
                add_jittered(x, y, z1, true, true, false);
            }
        }
        for (double z = z0; z <= z1; z += step)
        {
            for (double x = x0; x <= x1; x += step)
            {
                add_jittered(x, y0, z, true, false, true);
                add_jittered(x, y1, z, true, false, true);
            }
            for (double y = y0; y <= y1; y += step)
            {
                add_jittered(x0, y, z, false, true, true);
                add_jittered(x1, y, z, false, true, true);
            }
        }

        // Boxes {min corner, size}: only their side faces and top, as a lidar would see them.
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> boxes = {
            { { 4.0, 5.0, 0.0 }, { 1.0, 0.6, 1.2 } },
            { { 9.0, 8.0, 0.0 }, { 0.5, 1.5, 2.0 } },
            { { 11.5, 4.0, 0.0 }, { 1.2, 1.2, 0.8 } },
        };
        for (const auto& [corner, size] : boxes)
        {
            const Eigen::Vector3d c1 = corner + size;
            for (double z = corner.z(); z <= c1.z(); z += step)
            {
                for (double x = corner.x(); x <= c1.x(); x += step)
                {
                    add_jittered(x, corner.y(), z, true, false, true);
                    add_jittered(x, c1.y(), z, true, false, true);
                }
                for (double y = corner.y(); y <= c1.y(); y += step)
                {
                    add_jittered(corner.x(), y, z, false, true, true);
                    add_jittered(c1.x(), y, z, false, true, true);
                }
            }
            for (double x = corner.x(); x <= c1.x(); x += step)
            {
                for (double y = corner.y(); y <= c1.y(); y += step)
                {
                    add_jittered(x, y, c1.z(), true, true, false);
                }
            }
        }
        return points;
    }

    /**
     * @brief Pose from translation and roll/pitch/yaw.
     * @param t Translation [m].
     * @param roll_deg Rotation about X [deg].
     * @param pitch_deg Rotation about Y [deg].
     * @param yaw_deg Rotation about Z [deg].
     * @return The pose.
     */
    Eigen::Affine3d make_pose(const Eigen::Vector3d& t, double roll_deg, double pitch_deg, double yaw_deg)
    {
        const double d2r = M_PI / 180.0;
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translation() = t;
        pose.linear() =
            (Eigen::AngleAxisd(yaw_deg * d2r, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch_deg * d2r, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll_deg * d2r, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        return pose;
    }

    /** @brief Ground truth source-to-target pose of the test scans. */
    const Eigen::Affine3d true_pose = make_pose({ 0.4, -0.3, 0.1 }, 1.0, -1.0, 3.0);

    /** @brief Initial guess: the ground truth moved by about 0.2 m and 2 deg. */
    const Eigen::Affine3d initial_pose = true_pose * make_pose({ 0.15, -0.1, 0.05 }, 0.5, -0.5, 1.5);

    /**
     * @brief Source scan: every third room point, moved into the source frame, with 2 mm noise.
     * @param target The room in the target frame.
     * @return Points in the source frame.
     */
    std::vector<Eigen::Vector3d> make_source(const std::vector<Eigen::Vector3d>& target)
    {
        SplitMix64 rng(2);
        const Eigen::Affine3d target_to_source = true_pose.inverse();
        std::vector<Eigen::Vector3d> source;
        for (size_t i = 0; i < target.size(); i += 3)
        {
            // Separate statements, so the draws are in x, y, z order on every compiler (see make_room).
            const double noise_x = rng.uniform(-0.002, 0.002);
            const double noise_y = rng.uniform(-0.002, 0.002);
            const double noise_z = rng.uniform(-0.002, 0.002);
            const Eigen::Vector3d noise(noise_x, noise_y, noise_z);
            source.push_back(target_to_source * target[i] + noise);
        }
        return source;
    }

    /**
     * @brief Angle of the rotation between two poses.
     * @return Angle [deg].
     */
    double rotation_error_deg(const Eigen::Affine3d& a, const Eigen::Affine3d& b)
    {
        return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle() * 180.0 / M_PI;
    }
} // namespace

TEST_CASE("PairWiseICP::compute recovers a known offset, coarse to fine")
{
    const std::vector<Eigen::Vector3d> target = make_room();
    const std::vector<Eigen::Vector3d> source = make_source(target);

    Eigen::Affine3d pose = initial_pose;
    REQUIRE((pose.translation() - true_pose.translation()).norm() > 0.1);

    PairWiseICP icp;
    for (const double radius : { 1.0, 0.5, 0.25, 0.1 })
    {
        REQUIRE(icp.compute(source, target, radius, 30, pose));
    }

    CHECK((pose.translation() - true_pose.translation()).norm() < 0.005);
    CHECK(rotation_error_deg(pose, true_pose) < 0.05);
}

TEST_CASE("PairWiseICP::compute returns false and keeps the pose without correspondences")
{
    const std::vector<Eigen::Vector3d> target = make_room();
    std::vector<Eigen::Vector3d> source = make_source(target);
    for (auto& p : source)
    {
        p.x() += 100.0; // far outside the search radius of every target point
    }

    Eigen::Affine3d pose = initial_pose;
    PairWiseICP icp;
    CHECK_FALSE(icp.compute(source, target, 0.5, 5, pose));
    CHECK(pose.matrix() == initial_pose.matrix());
}

TEST_CASE("PairWiseICP::compute returns false with fewer than 100 correspondences")
{
    const std::vector<Eigen::Vector3d> target = make_room();
    std::vector<Eigen::Vector3d> source = make_source(target);
    source.resize(50);

    Eigen::Affine3d pose = initial_pose;
    PairWiseICP icp;
    CHECK_FALSE(icp.compute(source, target, 0.5, 5, pose));
    CHECK(pose.matrix() == initial_pose.matrix());
}

// Pins the exact result of a fixed run, so a refactor of compute() (threading, neighbour search, summation) is
// held to the numbers of the reference implementation (searching every point of each bucket, buckets ordered by
// point index). The tolerance covers rounding differences between compilers and platforms (FMA, libm, summation
// order: about 1e-16 here); a change to the algorithm moves the result far more (e.g. skipping the last point of
// each bucket: about 2e-4).
TEST_CASE("PairWiseICP::compute matches the reference result")
{
    const std::vector<Eigen::Vector3d> target = make_room();
    const std::vector<Eigen::Vector3d> source = make_source(target);

    Eigen::Affine3d pose = initial_pose;
    PairWiseICP icp;
    REQUIRE(icp.compute(source, target, 0.5, 10, pose));

    // Build with -DPRINT_REFERENCE to print new reference values after a deliberate change to the algorithm.
#ifdef PRINT_REFERENCE
    std::printf(
        "REFERENCE\n%.17g, %.17g, %.17g, %.17g\n%.17g, %.17g, %.17g, %.17g\n%.17g, %.17g, %.17g, %.17g\n",
        pose(0, 0),
        pose(0, 1),
        pose(0, 2),
        pose(0, 3),
        pose(1, 0),
        pose(1, 1),
        pose(1, 2),
        pose(1, 3),
        pose(2, 0),
        pose(2, 1),
        pose(2, 2),
        pose(2, 3));
#endif

    const double tolerance = 1e-9;
    // clang-format off
    const double expected[3][4] = {
        { 0.99847729809823327, -0.052634179800603725, -0.016514487553943396, 0.40002885380785902 },
        { 0.052329963103019038, 0.99846138157059194, -0.018342422789298693, -0.30000900062787589 },
        { 0.017454516438111307, 0.017450290222871295, 0.99969536721295715, 0.099966409536268633 },
    };
    // clang-format on
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 4; c++)
        {
            INFO("pose(", r, ", ", c, ") = ", pose(r, c), ", expected ", expected[r][c]);
            CHECK(std::abs(pose(r, c) - expected[r][c]) < tolerance);
        }
    }
}
