#pragma once

#include <RaylibWidgets/OrbitCamera.h>
#include <raymath.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace step1_render
{
    // Pick once on double click, using the same matrices and logical viewport as the displayed frame.
    // This searches projected pose positions, without rebuilding geometry or reading back GPU buffers.
    template<class Workers>
    std::optional<Vector3> nearestProjectedTrajectoryPoint(
        const Workers& workers, const Matrix& view, const Matrix& projection, Vector2 cursor, float width, float height)
    {
        if (width <= 0 || height <= 0 || !std::isfinite(cursor.x) || !std::isfinite(cursor.y))
            return std::nullopt;
        const Matrix m = MatrixMultiply(view, projection);
        float bestDistance = std::numeric_limits<float>::infinity();
        float bestDepth = std::numeric_limits<float>::infinity();
        std::optional<Vector3> result;
        for (const auto& worker : workers)
            for (const auto& pose : worker.intermediate_trajectory)
            {
                const auto p = pose.translation().template cast<float>().eval();
                const float w = m.m3 * p.x() + m.m7 * p.y() + m.m11 * p.z() + m.m15;
                if (!p.allFinite() || !std::isfinite(w) || w <= 1e-6f)
                    continue;
                const float x = (m.m0 * p.x() + m.m4 * p.y() + m.m8 * p.z() + m.m12) / w;
                const float y = (m.m1 * p.x() + m.m5 * p.y() + m.m9 * p.z() + m.m13) / w;
                const float z = (m.m2 * p.x() + m.m6 * p.y() + m.m10 * p.z() + m.m14) / w;
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || std::fabs(x) > 1 || std::fabs(y) > 1 || std::fabs(z) > 1)
                    continue;
                const float dx = (x * 0.5f + 0.5f) * width - cursor.x;
                const float dy = (0.5f - y * 0.5f) * height - cursor.y;
                const float distance = dx * dx + dy * dy;
                if (distance < bestDistance || (distance == bestDistance && z < bestDepth))
                {
                    bestDistance = distance;
                    bestDepth = z;
                    result = Vector3{ p.x(), p.y(), p.z() };
                }
            }
        return result;
    }

    // Only an explicit point selection detaches the pivot from the processing head.
    class TrajectoryFollow
    {
    public:
        float idleSeconds = 5.f;

        void reset(double now)
        {
            observed = wasProcessing = suspended = transitioning = recentering = automaticReturn = requested = false;
            selected.reset();
            lastNavigation = now;
        }

        void explore(Vector3 point, double now)
        {
            if (!finite(point))
                return;
            selected = point;
            suspended = transitioning = recentering = true;
            automaticReturn = requested = false;
            lastNavigation = now;
        }

        void returnToHead()
        {
            requested = true;
        }

        bool exploring() const
        {
            return suspended;
        }

        void update(
            raylib_widgets::OrbitCamera& camera,
            const std::optional<Vector3>& head,
            bool processing,
            bool interacting,
            bool focused,
            double now,
            float dt)
        {
            if (observed && state(camera) != previous)
            {
                lastNavigation = now;
                // Pan changes the framing, while orbit and zoom keep the same pivot.
                if (camera.euler.translate.x + camera.euler.rotationCenter.x != previous[2] + previous[5] ||
                    camera.euler.translate.y + camera.euler.rotationCenter.y != previous[3] + previous[6])
                    recentering = false;
                if (camera.isOrtho && (camera.orthoShiftX != previous[17] || camera.orthoShiftY != previous[18]))
                {
                    // Step 1's ortho view uses the Euler transform, not updateOrtho()'s lookAt.
                    const float dx = camera.orthoShiftX - previous[17], dy = camera.orthoShiftY - previous[18];
                    const float angle = (camera.euler.rotateX + camera.euler.rotateY) * DEG2RAD;
                    camera.euler.translate.x += std::cos(angle) * dx + std::sin(angle) * dy;
                    camera.euler.translate.y -= -std::sin(angle) * dx + std::cos(angle) * dy;
                    camera.eulerTransitionActive = false;
                    recentering = false;
                }
            }
            // Stop an automatic return when SLAM ends; explicit exploration/return may finish.
            if (wasProcessing && !processing && automaticReturn)
                transitioning = recentering = automaticReturn = false;
            wasProcessing = processing;
            camera.updateEulerTransition(dt);

            if (!processing || interacting || !focused)
                lastNavigation = now;
            if (requested && head && finite(*head))
            {
                suspended = automaticReturn = false;
                transitioning = recentering = true;
                selected.reset();
                camera.eulerTransitionActive = false;
            }
            else if (processing && suspended && now - lastNavigation >= idleSeconds && head && finite(*head))
            {
                suspended = false;
                transitioning = recentering = automaticReturn = true;
                selected.reset();
                camera.eulerTransitionActive = false;
            }
            requested = false;

            const auto target = suspended ? selected : head;
            if (focused && target && finite(*target) && (transitioning || (processing && !suspended)))
            {
                const float alpha = -std::expm1(-std::clamp(dt, 0.f, 0.1f) / (transitioning ? 0.15f : 0.35f));
                auto& e = camera.euler;
                const Vector3 oldCenter = e.rotationCenter;
                e.rotationCenter = Vector3Lerp(oldCenter, *target, alpha);
                const Vector3 delta = Vector3Subtract(e.rotationCenter, oldCenter);
                e.translate = Vector3Subtract(e.translate, delta); // preserve pan and zoom while the pivot moves
                const float dx = recentering ? -(e.translate.x + e.rotationCenter.x) * alpha : 0;
                const float dy = recentering ? -(e.translate.y + e.rotationCenter.y) * alpha : 0;
                e.translate.x += dx;
                e.translate.y += dy;
                if (camera.eulerTransitionActive)
                {
                    // Preserve camera-menu orientation transitions while following a moving head.
                    camera.eulerGoal.rotationCenter = e.rotationCenter;
                    camera.eulerGoal.translate = Vector3Subtract(camera.eulerGoal.translate, delta);
                    camera.eulerGoal.translate.x += dx;
                    camera.eulerGoal.translate.y += dy;
                }
                else
                    camera.eulerGoal = e;
                float error = Vector3Distance(e.rotationCenter, *target);
                if (recentering)
                    error =
                        std::max({ error, std::fabs(e.translate.x + e.rotationCenter.x), std::fabs(e.translate.y + e.rotationCenter.y) });
                if (error < 0.01f)
                    transitioning = recentering = automaticReturn = false;
            }
            previous = state(camera);
            observed = true;
        }

    private:
        static bool finite(Vector3 point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        static std::array<float, 22> state(const raylib_widgets::OrbitCamera& camera)
        {
            const auto& e = camera.euler;
            const auto& g = camera.eulerGoal;
            return { e.rotateX,
                     e.rotateY,
                     e.translate.x,
                     e.translate.y,
                     e.translate.z,
                     e.rotationCenter.x,
                     e.rotationCenter.y,
                     e.rotationCenter.z,
                     g.rotateX,
                     g.rotateY,
                     g.translate.x,
                     g.translate.y,
                     g.translate.z,
                     g.rotationCenter.x,
                     g.rotationCenter.y,
                     g.rotationCenter.z,
                     camera.orthoZoom,
                     camera.orthoShiftX,
                     camera.orthoShiftY,
                     camera.orthoZCenterH,
                     static_cast<float>(camera.isOrtho),
                     static_cast<float>(camera.lockZ) };
        }

        std::array<float, 22> previous{};
        std::optional<Vector3> selected;
        double lastNavigation = 0;
        bool observed = false;
        bool wasProcessing = false;
        bool suspended = false;
        bool transitioning = false;
        bool recentering = false;
        bool automaticReturn = false;
        bool requested = false;
    };
} // namespace step1_render
