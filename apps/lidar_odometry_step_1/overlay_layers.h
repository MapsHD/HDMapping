#pragma once

#include <memory>
#include <string>
#include <vector>

namespace step1_render
{
    //! Display-only cloud/trajectory layers, independent of the odometry state.
    class OverlayLayers
    {
    public:
        OverlayLayers();
        ~OverlayLayers();
        void queue(const std::vector<std::string>& files);
        void open();
        void gui();
        void draw();
        void shutdown();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace step1_render
