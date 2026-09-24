#pragma once

// Uploads a point cloud as several VAO/VBO pairs instead of one. raylib's
// rlLoadVertexBuffer() takes the byte size as int, so a single buffer
// overflows past 2 GiB (~76M points at 7 floats, ~134M at 4) and the
// following glDrawArrays reads from a buffer with no storage and crashes.
// Header-only (like Shaders.h) so core_raylib can use it without linking
// raylib_widgets.

#include "external/glad.h"
#include "rlgl.h"

#include <RaylibWidgets/PointBufferPart.h>

#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <vector>

namespace raylib_widgets
{

    //! 16M vertices -- at most 448 MB per buffer at 7 floats per vertex.
    constexpr size_t kMaxVerticesPerPart = size_t(1) << 24;

    //! Uploads interleaved float vertices, kMaxVerticesPerPart per part.
    //! @param data interleaved vertices, sum(attribSizes) floats each
    //! @param vertexCount number of vertices in data
    //! @param attribSizes float components per attribute, bound to locations 0, 1, ... in order
    //! @return the uploaded parts, empty when vertexCount is 0
    //! @note gl_VertexID restarts at 0 in every part.
    inline std::vector<PointBufferPart> uploadPointBufferParts(
        const float* data, size_t vertexCount, std::initializer_list<int> attribSizes)
    {
        int floatsPerVertex = 0;
        for (int s : attribSizes)
            floatsPerVertex += s;
        const int stride = floatsPerVertex * static_cast<int>(sizeof(float));

        std::vector<PointBufferPart> parts;
        for (size_t first = 0; first < vertexCount; first += kMaxVerticesPerPart)
        {
            PointBufferPart p;
            p.count = static_cast<int>(std::min(kMaxVerticesPerPart, vertexCount - first));
            p.vao = rlLoadVertexArray();
            rlEnableVertexArray(p.vao);
            p.vbo = rlLoadVertexBuffer(data + first * floatsPerVertex, p.count * stride, false);
            int location = 0;
            int offset = 0;
            for (int s : attribSizes)
            {
                rlSetVertexAttribute(location, s, RL_FLOAT, false, stride, offset * static_cast<int>(sizeof(float)));
                rlEnableVertexAttribute(location);
                ++location;
                offset += s;
            }
            rlDisableVertexArray();
            parts.push_back(p);
        }
        return parts;
    }

    //! Issues one glDrawArrays(GL_POINTS) per part; the caller binds the shader and sets its uniforms.
    inline void drawPointBufferParts(const std::vector<PointBufferPart>& parts)
    {
        for (const PointBufferPart& p : parts)
        {
            rlEnableVertexArray(p.vao);
            glDrawArrays(GL_POINTS, 0, p.count);
        }
        rlDisableVertexArray();
    }

    //! Releases every part's VAO/VBO and clears parts.
    inline void unloadPointBufferParts(std::vector<PointBufferPart>& parts)
    {
        for (const PointBufferPart& p : parts)
        {
            rlUnloadVertexArray(p.vao);
            rlUnloadVertexBuffer(p.vbo);
        }
        parts.clear();
    }

} // namespace raylib_widgets
