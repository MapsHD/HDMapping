#ifndef _GNSS_H_
#define _GNSS_H_

#include <string>
#include <vector>
#include <point_clouds.h>

class GNSS{
    public:

    struct GlobalPose{
        double timestamp;
        double lat;
        double lon;
        double alt;
        double hdop;
        double satelites_tracked;
        double height;
        double age;
        double time;
        double fix_quality;
        double x;
        double y;
    };

    GNSS(){;};
    ~GNSS(){;};

    //! \brief Load GNSS data from file and converts to PUWG92
    //! \param input_file_names - vector of file names
    //! \param localize - if true, the data is moved to the first point
    bool load(const std::vector<std::string> &input_file_names, bool localize = false);
    bool load_mercator_projection(const std::vector<std::string> &input_file_names);
    bool load_nmea_mercator_projection(const std::vector<std::string> &input_file_names);
    void render(const PointClouds &point_clouds_container);
    bool save_to_laz(const std::string &output_file_names, double offset_x, double offset_y, double offset_alt);

    std::vector<GlobalPose> gnss_poses;
    //double offset_x = 0;
    //double offset_y = 0;
    //double offset_alt = 0;
    bool show_correspondences = false;

    double WGS84ReferenceLatitude = 0;
    double WGS84ReferenceLongitude = 0;
    bool setWGS84ReferenceFromFirstPose = true;
};

//#pragma once
//#include <eigen3/Eigen/Core>
//#include <ceres/jet.h>
namespace GeographicalUtils
{
    using namespace Eigen;
    constexpr double earthSemimajorAxis = 6378137.0;
    constexpr double reciprocalFlattening = 1.0 / 298.257223563;
    constexpr double earthSemiminorAxis = earthSemimajorAxis * (1.0 - reciprocalFlattening);
    constexpr double firstEccentricitySquared = 2.0 * reciprocalFlattening - reciprocalFlattening * reciprocalFlattening;
    constexpr double secondEccentrictySquared =
        reciprocalFlattening * (2.0 - reciprocalFlattening) / ((1.0 - reciprocalFlattening) * (1.0 - reciprocalFlattening));

    template <typename T>
    T DegToRad(T degrees)
    {
        return degrees * T(M_PI / 180.0);
    }

    template <typename T>
    T RadToDeg(T radians)
    {
        return radians * T(180.0 / M_PI);
    }

    template <typename T>
    Eigen::Vector3<T> WGS84ToECEF(const Eigen::Vector3<T> &latitudeLongitudeAltitude)
    {
        //using namespace ceres;
        const T latitudeRad = DegToRad(latitudeLongitudeAltitude.x());
        const T longitudeRad = DegToRad(latitudeLongitudeAltitude.y());
        const T altitude = latitudeLongitudeAltitude.z();

        const T helper = sqrt(T(1.0) - firstEccentricitySquared * sin(latitudeRad) * sin(latitudeRad));

        const T X = (earthSemimajorAxis / helper + altitude) * cos(latitudeRad) * cos(longitudeRad);
        const T Y = (earthSemimajorAxis / helper + altitude) * cos(latitudeRad) * sin(longitudeRad);
        const T Z = (earthSemimajorAxis * (T(1.0) - firstEccentricitySquared) / helper + altitude) * sin(latitudeRad);

        return Eigen::Vector3<T>{X, Y, Z};
    }

    template <typename T>
    Eigen::Vector3<T> ECEFToENU(const Eigen::Vector3<T> &referenceLatitudeLongitudeAltitude, const Eigen::Vector3<T> ECEFPoint)
    {
        //using namespace ceres;
        const Vector3<T> referencePointInECEF = WGS84ToECEF(referenceLatitudeLongitudeAltitude);
        const Vector3<T> pointToReferencePointECEF = ECEFPoint - referencePointInECEF;

        const T referenceLatitudeRad = DegToRad(referenceLatitudeLongitudeAltitude.x());  // m_latitude
        const T referenceLongitudeRad = DegToRad(referenceLatitudeLongitudeAltitude.y()); // m_longitude
        return {-sin(referenceLongitudeRad) * pointToReferencePointECEF.x() + cos(referenceLongitudeRad) * pointToReferencePointECEF.y(),
                -sin(referenceLatitudeRad) * cos(referenceLongitudeRad) * pointToReferencePointECEF.x() -
                    sin(referenceLatitudeRad) * sin(referenceLongitudeRad) * pointToReferencePointECEF.y() +
                    cos(referenceLatitudeRad) * pointToReferencePointECEF.z(),
                cos(referenceLatitudeRad) * cos(referenceLongitudeRad) * pointToReferencePointECEF.x() +
                    cos(referenceLatitudeRad) * sin(referenceLongitudeRad) * pointToReferencePointECEF.y() +
                    sin(referenceLatitudeRad) * pointToReferencePointECEF.z()};
    }

    template <typename T>
    Vector3<T> ENUToECEF(const Vector3<T> &referenceLatitudeLongitudeAltitude, const Vector3<T> &ENUPoint)
    {
        //using namespace ceres;
        const auto referenceECEF = WGS84ToECEF(referenceLatitudeLongitudeAltitude);

        const T referenceLatitudeRad = DegToRad(referenceLatitudeLongitudeAltitude.x());
        const T referenceLongitudeRad = DegToRad(referenceLatitudeLongitudeAltitude.y());
        const T &e = ENUPoint.x();
        const T &n = ENUPoint.y();
        const T &u = ENUPoint.z();

        return {-sin(referenceLongitudeRad) * e - cos(referenceLongitudeRad) * sin(referenceLatitudeRad) * n +
                    cos(referenceLongitudeRad) * cos(referenceLatitudeRad) * u + referenceECEF.x(),
                cos(referenceLongitudeRad) * e - sin(referenceLongitudeRad) * sin(referenceLatitudeRad) * n +
                    cos(referenceLatitudeRad) * sin(referenceLongitudeRad) * u + referenceECEF.y(),
                cos(referenceLatitudeRad) * n + sin(referenceLatitudeRad) * u + referenceECEF.z()};
    }

    template <typename T>
    Vector3<T> ECEFToWGS84(const Vector3<T> &ECFEPoint)
    {
        //using namespace ceres;
        const T &x = ECFEPoint.x();
        const T &y = ECFEPoint.y();
        const T &z = ECFEPoint.z();

        const T radiusSquared = x * x + y * y;
        const T radius = sqrt(radiusSquared);

        const T E2 = earthSemimajorAxis * earthSemimajorAxis - earthSemiminorAxis * earthSemiminorAxis;
        const T F = 54.0 * earthSemiminorAxis * earthSemiminorAxis * z * z;
        const T G = radiusSquared + (1.0 - firstEccentricitySquared) * z * z - firstEccentricitySquared * E2;
        const T c = (firstEccentricitySquared * firstEccentricitySquared * F * radiusSquared) / (G * G * G);
        const T s = pow(1. + c + sqrt(c * c + 2. * c), 1. / 3);
        const T P = F / (3.0 * (s + 1.0 / s + 1.0) * (s + 1.0 / s + 1.0) * G * G);
        const T Q = sqrt(1.0 + 2.0 * firstEccentricitySquared * firstEccentricitySquared * P);

        const T ro = -(firstEccentricitySquared * P * radius) / (1.0 + Q) +
                     sqrt(
                         (earthSemimajorAxis * earthSemimajorAxis / 2.0) * (1.0 + 1.0 / Q) -
                         ((1.0 - firstEccentricitySquared) * P * z * z) / (Q * (1.0 + Q)) - P * radiusSquared / 2.0);
        const T tmp = (radius - firstEccentricitySquared * ro) * (radius - firstEccentricitySquared * ro);
        const T U = sqrt(tmp + z * z);
        const T V = sqrt(tmp + (1.0 - firstEccentricitySquared) * z * z);
        const T zo = (earthSemiminorAxis * earthSemiminorAxis * z) / (earthSemimajorAxis * V);

        const T latitude = atan((z + secondEccentrictySquared * zo) / radius);
        const T longitude = atan2(y, x);
        const T altitude = U * (1.0 - earthSemiminorAxis * earthSemiminorAxis / (earthSemimajorAxis * V));

        return {RadToDeg(latitude), RadToDeg(longitude), altitude};
    }

    template <typename T>
    Eigen::Vector3<T> LevelToWGS84(const Eigen::Vector3<T> levelTiePoint,
                                   const Eigen::Quaternion<T> enuLevelRotation,
                                   const Eigen::Vector3<T> &enuLevelTranslation,
                                   const Eigen::Vector3<T> EnuLatLonReference)
    {
        const Eigen::Vector3<T> levelTiePointEnu = enuLevelRotation * levelTiePoint + enuLevelTranslation;
        const Eigen::Vector3<T> levelTiePointEcef = ENUToECEF(EnuLatLonReference, levelTiePointEnu);
        return ECEFToWGS84(levelTiePointEcef);
    }

    template <typename T>
    Eigen::Vector3<T> WGS84ToLevel(const Eigen::Vector3<T> wqs84TiePoint,
                                   const Eigen::Quaternion<T> enuLevelRotation,
                                   const Eigen::Vector3<T> &enuLevelTranslation,
                                   const Eigen::Vector3<T> EnuLatLonReference)
    {

        const auto ecef = WGS84ToECEF(wqs84TiePoint);
        const auto enu = ECEFToENU(EnuLatLonReference, ecef);
        const Eigen::Quaternion<T> enuRotationQuatInverse = enuLevelRotation.inverse();
        const Eigen::Vector3<T> enuOriginLocationGoldInverse = -(enuRotationQuatInverse * enuLevelTranslation);
        return enuRotationQuatInverse * enu + enuOriginLocationGoldInverse;
    }

    template <typename T>
    Eigen::Vector3<T> ECEFCostFunction(const Eigen::Vector3<T> levelTiePoint,
                                       const Eigen::Vector3<T> ecefTiePoint,
                                       const Eigen::Quaternion<T> enuLevelRotation,
                                       const Eigen::Vector3<T> &enuLevelTranslation,
                                       const Eigen::Vector3<T> LatLonReference)
    {
        const Eigen::Vector3<T> levelTiePointEnu = enuLevelRotation * levelTiePoint + enuLevelTranslation;
        const Eigen::Vector3<T> levelTiePointEcef = ENUToECEF(LatLonReference, levelTiePointEnu);
        return (levelTiePointEcef - ecefTiePoint);
    }
}

#endif