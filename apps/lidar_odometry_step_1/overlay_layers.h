#pragma once

#include <memory>
#include <string>
#include <vector>

namespace step1_render
{
    //! GUI-owned cloud/trajectory overlays; background imports never access odometry state.
    class OverlayLayers
    {
    public:
        OverlayLayers();
        ~OverlayLayers();
        //! Queue imports for confirmation in gui(); existing layers survive failed or cancelled imports.
        void queue(const std::vector<std::string>& files);
        void open();
        void gui();
        void draw();
        //! Cancel pending work and release GPU resources before closing the graphics context.
        void shutdown();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace step1_render
