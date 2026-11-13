#include <RaylibWidgets/Timeline.h>

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <string>

namespace raylib_widgets
{

    namespace
    {
        //! Smallest "round" tick step (s) that keeps ticks at least `minPx` apart.
        double tickStep(double durationSec, float widthPx, float minPx)
        {
            static const double kSteps[] = { 0.1, 0.2, 0.5, 1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600 };
            const double pxPerSec = widthPx / durationSec;
            for (double step : kSteps)
                if (step * pxPerSec >= minPx)
                    return step;
            double step = kSteps[std::size(kSteps) - 1];
            while (step * pxPerSec < minPx)
                step *= 2.0;
            return step;
        }

        //! Tick label: "m:ss" from a minute on, else seconds with a decimal only when
        //! the tick step needs one.
        std::string tickLabel(double sec, double step)
        {
            char buf[32];
            if (sec >= 60.0)
                std::snprintf(buf, sizeof(buf), "%d:%02d", int(sec + 1e-6) / 60, int(sec + 1e-6) % 60);
            else if (step < 1.0)
                std::snprintf(buf, sizeof(buf), "%.1fs", sec);
            else
                std::snprintf(buf, sizeof(buf), "%.0fs", sec);
            return buf;
        }
    } // namespace

    bool timeline(const char* id, float& progress, double durationSec, float height)
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const float w = std::max(1.f, ImGui::GetContentRegionAvail().x);
        const float trackH = height - ImGui::GetTextLineHeight();
        const ImVec2 p1(p0.x + w, p0.y + trackH);

        ImGui::InvisibleButton(id, ImVec2(w, height));
        bool moved = false;
        if (ImGui::IsItemActive())
        {
            const float f = std::clamp((ImGui::GetIO().MousePos.x - p0.x) / w, 0.f, 1.f);
            moved = f != progress;
            progress = f;
        }
        if (ImGui::IsItemHovered() && durationSec > 0.0)
        {
            const float f = std::clamp((ImGui::GetIO().MousePos.x - p0.x) / w, 0.f, 1.f);
            ImGui::SetTooltip("%.2f s", f * durationSec);
        }

        dl->AddRectFilled(p0, p1, IM_COL32(40, 40, 40, 255), 3.f);
        dl->AddRectFilled(p0, ImVec2(p0.x + w * progress, p1.y), IM_COL32(60, 110, 160, 255), 3.f, ImDrawFlags_RoundCornersLeft);

        if (durationSec > 0.0)
        {
            // Major ticks carry a label; four unlabelled minor ticks between them.
            const double major = tickStep(durationSec, w, 70.f);
            const double minor = major / 5.0;
            const double pxPerSec = w / durationSec;
            for (int k = 0; k * minor <= durationSec + 1e-9; ++k)
            {
                const bool isMajor = k % 5 == 0;
                const float x = p0.x + float(k * minor * pxPerSec);
                dl->AddLine(ImVec2(x, p1.y - trackH * (isMajor ? 0.6f : 0.3f)), ImVec2(x, p1.y), IM_COL32(170, 170, 170, 255));
                if (isMajor)
                {
                    const std::string label = tickLabel(k * minor, major);
                    const float lw = ImGui::CalcTextSize(label.c_str()).x;
                    const float lx = std::clamp(x - lw * 0.5f, p0.x, p0.x + w - lw);
                    dl->AddText(ImVec2(lx, p1.y), IM_COL32(200, 200, 200, 255), label.c_str());
                }
            }
        }

        const float px = p0.x + w * progress;
        const ImU32 playheadCol = IM_COL32(255, 220, 60, 255);
        dl->AddLine(ImVec2(px, p0.y), ImVec2(px, p1.y), playheadCol, 2.f);
        dl->AddTriangleFilled(ImVec2(px - 5.f, p0.y), ImVec2(px + 5.f, p0.y), ImVec2(px, p0.y + 6.f), playheadCol);
        return moved;
    }

} // namespace raylib_widgets
