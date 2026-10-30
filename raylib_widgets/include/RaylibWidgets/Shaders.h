#pragma once

// Shared GLSL fragment-shader snippets, string-spliced into each app's own
// otherwise-distinct point-cloud shaders. Header-only (no .cpp, nothing to
// link) -- consumers just need this directory on their include path, which
// core_raylib gets via a plain target_include_directories (not a full
// raylib_widgets link, to avoid pulling imgui_raylib into core_raylib as a
// new transitive dependency for what's just a compile-time string constant).
namespace raylib_widgets
{

    inline constexpr const char* kColormapNames[] = { "Jet",   "Viridis",   "Turbo", "Inferno", "Plasma",        "Cividis",
                                                      "Magma", "Cubehelix", "Hot",   "Blues",   "Blue-White-Red" };

    // Colormaps, t in [0,1]. "jet" (blue -> cyan -> yellow -> red) was
    // byte-for-byte duplicated in core/src/raylib_render.cpp's ScanRenderer,
    // apps/camera_lidar_calibration/Renderer.cpp's two point shaders, and
    // apps/camera_lidar_trajectory_viewer/TrajectoryViewer.cpp's point shader.
    // Splice into a fragment shader's source via string concatenation, e.g.:
    //   static const std::string kMyFS = std::string(R"(#version 330
    //   ...uniforms/varyings...
    //   )") + raylib_widgets::kColormapsGLSL + R"(
    //   void main() { ... }
    //   )";
    inline constexpr const char* kColormapsGLSL = R"(
vec3 jet(float t) {
    t = clamp(t, 0.0, 1.0);
    return clamp(vec3(1.5 - abs(4.0*t - 3.0),
                      1.5 - abs(4.0*t - 2.0),
                      1.5 - abs(4.0*t - 1.0)), 0.0, 1.0);
}
vec3 poly6(float t, vec3 c0, vec3 c1, vec3 c2, vec3 c3, vec3 c4, vec3 c5, vec3 c6) {
    return c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6)))));
}
vec3 viridis(float t) {
    return poly6(t,
        vec3(0.2777273272234177, 0.005407344544966578, 0.3340998053353061),
        vec3(0.1050930431085774, 1.404613529898575, 1.384590162594685),
        vec3(-0.3308618287255563, 0.214847559468213, 0.09509516302823659),
        vec3(-4.634230498983486, -5.799100973351585, -19.33244095627987),
        vec3(6.228269936347081, 14.17993336680509, 56.69055260068105),
        vec3(4.776384997670288, -13.74514537774601, -65.35303263337234),
        vec3(-5.435455855934631, 4.645852612178535, 26.3124352495832));
}
vec3 turbo(float t) {
    const vec4 redLow = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    const vec4 greenLow = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    const vec4 blueLow = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
    const vec2 redHigh = vec2(-152.94239396, 59.28637943);
    const vec2 greenHigh = vec2(4.27729857, 2.82956604);
    const vec2 blueHigh = vec2(-89.90310912, 27.34824973);
    vec4 powersLow = vec4(1.0, t, t * t, t * t * t);
    vec2 powersHigh = powersLow.zw * powersLow.z;
    return vec3(dot(powersLow, redLow) + dot(powersHigh, redHigh),
                dot(powersLow, greenLow) + dot(powersHigh, greenHigh),
                dot(powersLow, blueLow) + dot(powersHigh, blueHigh));
}
vec3 inferno(float t) {
    return poly6(t,
        vec3(0.0002189403691192265, 0.001651004631001012, -0.01948089843709184),
        vec3(0.1065134194856116, 0.5639564367884091, 3.932712388889277),
        vec3(11.60249308247187, -3.972853965665698, -15.9423941062914),
        vec3(-41.70399613139459, 17.43639888205313, 44.35414519872813),
        vec3(77.162935699427, -33.40235894210092, -81.80730925738993),
        vec3(-71.31942824499214, 32.62606426397723, 73.20951985803202),
        vec3(25.13112622477341, -12.24266895238567, -23.07032500287172));
}
vec3 plasma(float t) {
    return poly6(t,
        vec3(0.05873234392399702, 0.02333670892565664, 0.5433401826748754),
        vec3(2.176514634195958, 0.2383834171260182, 0.7539604599784036),
        vec3(-2.689460476458034, -7.455851135738909, 3.110799939717086),
        vec3(6.130348345893603, 42.3461881477227, -28.51885465332158),
        vec3(-11.10743619062271, -82.66631109428045, 60.13984767418263),
        vec3(10.02306557647065, 71.41361770095349, -54.07218655560067),
        vec3(-3.658713842777788, -22.93153465461149, 18.19190778539828));
}
vec3 cividis(float t) {
    const vec3 stops[10] = vec3[10](
        vec3(0.0000, 0.1333, 0.3059),
        vec3(0.0706, 0.2078, 0.4392),
        vec3(0.2314, 0.2863, 0.4235),
        vec3(0.3412, 0.3647, 0.4275),
        vec3(0.4392, 0.4431, 0.4510),
        vec3(0.5412, 0.5255, 0.4706),
        vec3(0.6471, 0.6118, 0.4549),
        vec3(0.7647, 0.7020, 0.4118),
        vec3(0.8824, 0.8000, 0.3333),
        vec3(0.9961, 0.9098, 0.2196));
    float scaled = t * 9.0;
    int index = min(int(scaled), 8);
    return mix(stops[index], stops[index + 1], scaled - float(index));
}
vec3 magma(float t) {
    return poly6(t,
        vec3(-0.002136485053939582, -0.000749655052795221, -0.005386127855323933),
        vec3(0.2516605407371642, 0.6775232436837668, 2.494026599312351),
        vec3(8.353717279216625, -3.577719514958484, 0.3144679030132573),
        vec3(-27.66873308576866, 14.26473078096533, -13.64921318813922),
        vec3(52.17613981234068, -27.94360607168351, 12.94416944238394),
        vec3(-50.76852536473588, 29.04658282127291, 4.23415299384598),
        vec3(18.65570506591883, -11.48977351997711, -5.601961508734096));
}
vec3 cubehelix(float t) {
    const float start = 0.5;
    const float rotations = -1.5;
    const float hue = 1.0;
    const float twoPi = 6.283185307179586;
    float amplitude = hue * t * (1.0 - t) * 0.5;
    float angle = twoPi * (start / 3.0 + rotations * t);
    float cosine = cos(angle);
    float sine = sin(angle);
    return t + amplitude * vec3(-0.14861 * cosine + 1.78277 * sine,
                                -0.29227 * cosine - 0.90649 * sine,
                                1.97294 * cosine);
}
vec3 hot(float t) {
    const vec3 rampBegin = vec3(0.0, 0.365079, 0.746032);
    const vec3 rampEnd = vec3(0.365079, 0.746032, 1.0);
    vec3 color = clamp((vec3(t) - rampBegin) / (rampEnd - rampBegin), 0.0, 1.0);
    color.r = mix(0.0416, 1.0, color.r);
    return color;
}
vec3 blues(float t) {
    const vec3 stops[9] = vec3[9](
        vec3(0.9686, 0.9843, 1.0000),
        vec3(0.8706, 0.9216, 0.9686),
        vec3(0.7765, 0.8588, 0.9373),
        vec3(0.6196, 0.7922, 0.8824),
        vec3(0.4196, 0.6824, 0.8392),
        vec3(0.2588, 0.5725, 0.7765),
        vec3(0.1294, 0.4431, 0.7098),
        vec3(0.0314, 0.3176, 0.6118),
        vec3(0.0314, 0.1882, 0.4196));
    float scaled = t * 8.0;
    int index = min(int(scaled), 7);
    return mix(stops[index], stops[index + 1], scaled - float(index));
}
vec3 blueWhiteRed(float t) {
    if (t < 0.5) return mix(vec3(0.0, 0.0, 1.0), vec3(1.0), t * 2.0);
    return mix(vec3(1.0), vec3(1.0, 0.0, 0.0), t * 2.0 - 1.0);
}
vec3 applyColormap(int index, float t) {
    t = clamp(t, 0.0, 1.0);
    switch (index) {
        case 1: return viridis(t);
        case 2: return turbo(t);
        case 3: return inferno(t);
        case 4: return plasma(t);
        case 5: return cividis(t);
        case 6: return magma(t);
        case 7: return cubehelix(t);
        case 8: return hot(t);
        case 9: return blues(t);
        case 10: return blueWhiteRed(t);
        default: return jet(t);
    }
}
)";

} // namespace raylib_widgets
