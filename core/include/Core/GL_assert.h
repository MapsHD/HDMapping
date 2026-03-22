#pragma once

#include <GL/glew.h>

#include <cassert>
#include <iostream>

// Macro to wrap OpenGL calls and check for errors
#define GL_CALL(x)                                                                                                                         \
    do                                                                                                                                     \
    {                                                                                                                                      \
        x;                                                                                                                                 \
        GLenum err = glGetError();                                                                                                         \
        if (err != GL_NO_ERROR)                                                                                                            \
        {                                                                                                                                  \
            std::cerr << "OpenGL Error (" << err << "): " << __FILE__ << " at line " << __LINE__ << std::endl;                             \
            assert(false && "OpenGL Error");                                                                                               \
        }                                                                                                                                  \
    } while (false)

// Macro to wrap OpenGL calls that return a value. Evaluates the call once, checks for GL errors,
// and returns the call result. Implemented as a lambda to preserve the expression type.
#define GL_CALL_RET(x)                                                                                                                     \
    (                                                                                                                                      \
        [&]() -> decltype(x)                                                                                                               \
        {                                                                                                                                  \
            auto _gl_call_result = (x);                                                                                                    \
            GLenum _gl_call_err = glGetError();                                                                                            \
            if (_gl_call_err != GL_NO_ERROR)                                                                                               \
            {                                                                                                                              \
                std::cerr << "OpenGL Error (" << _gl_call_err << "): " << __FILE__ << " at line " << __LINE__ << std::endl;                \
                assert(false && "OpenGL Error");                                                                                           \
            }                                                                                                                              \
            return _gl_call_result;                                                                                                        \
        }())

static bool CheckUniformLocation(GLint location, const char* name)
{
    if (location == GL_INVALID_VALUE)
    {
        std::cerr << "Uniform " << name << " GL_INVALID_VALUE" << std::endl;
    }
    if (location == GL_INVALID_OPERATION)
    {
        std::cerr << "Uniform " << name << " GL_INVALID_OPERATION" << std::endl;
    }
    assert(location != GL_INVALID_VALUE && location != GL_INVALID_OPERATION);
    return location != GL_INVALID_VALUE && location != GL_INVALID_OPERATION;
}
// Helper function to convert OpenGL error code to string (optional, for more readable errors)
static const char* GetGLErrorString(GLenum error)
{
    switch (error)
    {
    case GL_NO_ERROR:
        return "No error";
    case GL_INVALID_ENUM:
        return "Invalid enum";
    case GL_INVALID_VALUE:
        return "Invalid value";
    case GL_INVALID_OPERATION:
        return "Invalid operation";
    case GL_STACK_OVERFLOW:
        return "Stack overflow";
    case GL_STACK_UNDERFLOW:
        return "Stack underflow";
    case GL_OUT_OF_MEMORY:
        return "Out of memory";
    default:
        return "Unknown error";
    }
}
