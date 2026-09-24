#pragma once

// Kept apart from PointBufferParts.h so headers can hold parts without pulling in glad.h.
namespace raylib_widgets
{

    //! One VAO/VBO pair holding a contiguous slice of a point cloud.
    struct PointBufferPart
    {
        unsigned int vao = 0;
        unsigned int vbo = 0;
        int count = 0;
    };

} // namespace raylib_widgets
