/*
 * MIT License
 *
 * Copyright (c) 2018  Christian Berger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef WGS84TOCARTESIAN_HPP
#define WGS84TOCARTESIAN_HPP

#include <cmath>
#include <array>
#include <limits>

namespace wgs84 {

/**
 * @param WGS84Reference WGS84 position to be used as reference.
 * @param WGS84Position WGS84 position to be transformed.
 * @return std::array<double, 2> Cartesian position after transforming WGS84Position using the given WGS84Reference using Mercator projection.
 */
inline std::array<double, 2> toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position) {
#ifndef M_PI
    constexpr double M_PI = 3.141592653589793;
#endif
    constexpr double DEG_TO_RAD{M_PI / 180.0};
    constexpr double HALF_PI{M_PI / 2.0};
    constexpr double EPSILON10{1.0e-10};
    constexpr double EPSILON12{1.0e-12};

    constexpr double EQUATOR_RADIUS{6378137.0};
    constexpr double FLATTENING{1.0 / 298.257223563};
    constexpr double SQUARED_ECCENTRICITY{2.0 * FLATTENING - FLATTENING * FLATTENING};
    constexpr double SQUARE_ROOT_ONE_MINUS_ECCENTRICITY{0.996647189335};
    constexpr double POLE_RADIUS{EQUATOR_RADIUS * SQUARE_ROOT_ONE_MINUS_ECCENTRICITY};

    constexpr double C00{1.0};
    constexpr double C02{0.25};
    constexpr double C04{0.046875};
    constexpr double C06{0.01953125};
    constexpr double C08{0.01068115234375};
    constexpr double C22{0.75};
    constexpr double C44{0.46875};
    constexpr double C46{0.01302083333333333333};
    constexpr double C48{0.00712076822916666666};
    constexpr double C66{0.36458333333333333333};
    constexpr double C68{0.00569661458333333333};
    constexpr double C88{0.3076171875};

    constexpr double R0{C00 - SQUARED_ECCENTRICITY * (C02 + SQUARED_ECCENTRICITY * (C04 + SQUARED_ECCENTRICITY * (C06 + SQUARED_ECCENTRICITY * C08)))};
    constexpr double R1{SQUARED_ECCENTRICITY * (C22 - SQUARED_ECCENTRICITY * (C04 + SQUARED_ECCENTRICITY * (C06 + SQUARED_ECCENTRICITY * C08)))};
    constexpr double R2T{SQUARED_ECCENTRICITY * SQUARED_ECCENTRICITY};
    constexpr double R2{R2T * (C44 - SQUARED_ECCENTRICITY * (C46 + SQUARED_ECCENTRICITY * C48))};
    constexpr double R3T{R2T * SQUARED_ECCENTRICITY};
    constexpr double R3{R3T * (C66 - SQUARED_ECCENTRICITY * C68)};
    constexpr double R4{R3T * SQUARED_ECCENTRICITY * C88};

    auto mlfn = [&](const double &lat) {
        const double sin_phi{std::sin(lat)};
        const double cos_phi{std::cos(lat) * sin_phi};
        const double squared_sin_phi = sin_phi * sin_phi;
        return (R0 * lat - cos_phi * (R1 + squared_sin_phi * (R2 + squared_sin_phi * (R3 + squared_sin_phi * R4))));
    };

    const double ML0{mlfn(WGS84Reference[0] * DEG_TO_RAD)};

    auto msfn = [&](const double &sinPhi, const double &cosPhi, const double &es) { return (cosPhi / std::sqrt(1.0 - es * sinPhi * sinPhi)); };

    auto project = [&](double lat, double lon) {
        std::array<double, 2> retVal{lon, -1.0 * ML0};
        if (!(std::abs(lat) < EPSILON10)) {
            const double ms{(std::abs(std::sin(lat)) > EPSILON10) ? msfn(std::sin(lat), std::cos(lat), SQUARED_ECCENTRICITY) / std::sin(lat) : 0.0};
            retVal[0] = ms * std::sin(lon *= std::sin(lat));
            retVal[1] = (mlfn(lat) - ML0) + ms * (1.0 - std::cos(lon));
        }
        return retVal;
    };

    auto fwd = [&](double lat, double lon) {
        const double D = std::abs(lat) - HALF_PI;
        if ((D > EPSILON12) || (std::abs(lon) > 10.0)) {
            return std::array<double, 2>{0.0, 0.0};
        }
        if (std::abs(D) < EPSILON12) {
            lat = (lat < 0.0) ? -1.0 * HALF_PI : HALF_PI;
        }
        lon -= WGS84Reference[1] * DEG_TO_RAD;
        const auto projectedRetVal{project(lat, lon)};
        return std::array<double, 2>{EQUATOR_RADIUS * projectedRetVal[0], EQUATOR_RADIUS * projectedRetVal[1]};
    };

    return fwd(WGS84Position[0] * DEG_TO_RAD, WGS84Position[1] * DEG_TO_RAD);
}

/**
 * @param WGS84Reference WGS84 position to be used as reference.
 * @param CartesianPosition Cartesian position to be transformed.
 * @return std::array<double, 2> Approximating a WGS84 position from a given CartesianPosition based on a given WGS84Reference using Mercator projection.
 */
inline std::array<double, 2> fromCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &CartesianPosition) {
    constexpr double EPSILON10{1.0e-4};
    const int32_t signLon{(CartesianPosition[0] < 0) ? -1 : 1};
    const int32_t signLat{(CartesianPosition[1] < 0) ? -1 : 1};

    std::array<double, 2> approximateWGS84Position{WGS84Reference};
    std::array<double, 2> cartesianResult{toCartesian(WGS84Reference, approximateWGS84Position)};

    double dPrev{(std::numeric_limits<double>::max)()};
    double d = std::abs(CartesianPosition[1] - cartesianResult[1]);
    double incLat{1e-6};
    while ((d < dPrev) && (d > EPSILON10)) {
        incLat = std::max(1e-6 * d, static_cast<double>(1e-9));
        approximateWGS84Position[0] = approximateWGS84Position[0] + signLat * incLat;
        cartesianResult             = toCartesian(WGS84Reference, approximateWGS84Position);
        dPrev                       = d;
        d                           = std::abs(CartesianPosition[1] - cartesianResult[1]);
    }

    dPrev = (std::numeric_limits<double>::max)();
    d     = std::abs(CartesianPosition[0] - cartesianResult[0]);
    double incLon{1e-6};
    while ((d < dPrev) && (d > EPSILON10)) {
        incLon = std::max(1e-6 * d, static_cast<double>(1e-9));
        approximateWGS84Position[1] = approximateWGS84Position[1] + signLon * incLon;
        cartesianResult             = toCartesian(WGS84Reference, approximateWGS84Position);
        dPrev                       = d;
        d                           = std::abs(CartesianPosition[0] - cartesianResult[0]);
    }

    return approximateWGS84Position;
}
}
#endif
