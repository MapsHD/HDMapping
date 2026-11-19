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

    bool timeline(
        const char* id,
        float& progress,
        double durationSec,
        float height,
        const std::vector<TimelineSpan>& spans,
        const std::vector<TimelineLane>& lanes)
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const float w = std::max(1.f, ImGui::GetContentRegionAvail().x);
        const float trackH = height - ImGui::GetTextLineHeight();
        const ImVec2 p1(p0.x + w, p0.y + trackH);
        // Lanes sit between the track and the tick labels.
        const float lanesBottom = p1.y + kTimelineLaneHeight * static_cast<float>(lanes.size());
        auto toX = [&](float f)
        {
            return p0.x + w * std::clamp(f, 0.f, 1.f);
        };

        ImGui::InvisibleButton(id, ImVec2(w, height + (lanesBottom - p1.y)));
        bool moved = false;
        if (ImGui::IsItemActive())
        {
            const float f = std::clamp((ImGui::GetIO().MousePos.x - p0.x) / w, 0.f, 1.f);
            moved = f != progress;
            progress = f;
        }
        if (ImGui::IsItemHovered() && durationSec > 0.0)
        {
            const ImVec2 mouse = ImGui::GetIO().MousePos;
            const float f = std::clamp((mouse.x - p0.x) / w, 0.f, 1.f);
            char text[256];
            std::snprintf(text, sizeof(text), "%.2f s", f * durationSec);
            std::string tip = text;
            if (mouse.y < p1.y)
            {
                for (const TimelineSpan& span : spans)
                    if (!span.label.empty() && f >= span.from && f <= span.to)
                        tip += ", " + span.label;
            }
            else if (mouse.y < lanesBottom)
            {
                const auto& bars = lanes[std::min(lanes.size() - 1, static_cast<size_t>((mouse.y - p1.y) / kTimelineLaneHeight))].bars;
                for (size_t k = 0; k < bars.size(); ++k)
                {
                    if (f >= bars[k].from && f <= bars[k].to)
                    {
                        if (!bars[k].label.empty())
                            tip += "\n" + bars[k].label;
                        break;
                    }
                    if (k + 1 < bars.size() && f > bars[k].to && f < bars[k + 1].from)
                    {
                        std::snprintf(text, sizeof(text), "\ngap of %.3f s", (bars[k + 1].from - bars[k].to) * durationSec);
                        tip += text;
                        break;
                    }
                }
            }
            ImGui::SetTooltip("%s", tip.c_str());
        }

        dl->AddRectFilled(p0, p1, IM_COL32(40, 40, 40, 255), 3.f);
        dl->AddRectFilled(p0, ImVec2(p0.x + w * progress, p1.y), IM_COL32(60, 110, 160, 255), 3.f, ImDrawFlags_RoundCornersLeft);
        // At least a pixel wide, so a short span still shows on a long timeline.
        for (const TimelineSpan& span : spans)
        {
            const float x0 = toX(span.from);
            dl->AddRectFilled(ImVec2(x0, p0.y), ImVec2(std::max(x0 + 1.f, toX(span.to)), p1.y), span.color);
        }

        for (size_t l = 0; l < lanes.size(); ++l)
        {
            const float y0 = p1.y + kTimelineLaneHeight * static_cast<float>(l) + 1.f;
            const float y1 = y0 + kTimelineLaneHeight - 1.f;
            dl->AddRectFilled(ImVec2(p0.x, y0), ImVec2(p1.x, y1), IM_COL32(30, 30, 30, 255));
            const auto& bars = lanes[l].bars;
            for (size_t k = 0; k < bars.size(); ++k)
            {
                if (bars[k].to < 0.f || bars[k].from > 1.f)
                    continue;
                const float x0 = toX(bars[k].from);
                float x1 = std::max(x0 + 1.f, toX(bars[k].to));
                // A gap narrower than a pixel would only show where it straddles a pixel centre.
                if (k + 1 < bars.size() && toX(bars[k + 1].from) - x1 < 1.f)
                    x1 = std::max(x1, toX(bars[k + 1].from));
                dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), bars[k].color);
            }
            if (!lanes[l].name.empty())
                dl->AddText(
                    ImGui::GetFont(),
                    kTimelineLaneHeight - 1.f,
                    ImVec2(p0.x + 3.f, y0),
                    IM_COL32(255, 255, 255, 230),
                    lanes[l].name.c_str());
        }

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
                    dl->AddText(ImVec2(lx, lanesBottom), IM_COL32(200, 200, 200, 255), label.c_str());
                }
            }
        }

        const float px = p0.x + w * progress;
        const ImU32 playheadCol = IM_COL32(255, 220, 60, 255);
        dl->AddLine(ImVec2(px, p0.y), ImVec2(px, lanesBottom), playheadCol, 2.f);
        dl->AddTriangleFilled(ImVec2(px - 5.f, p0.y), ImVec2(px + 5.f, p0.y), ImVec2(px, p0.y + 6.f), playheadCol);
        return moved;
    }

} // namespace raylib_widgets
