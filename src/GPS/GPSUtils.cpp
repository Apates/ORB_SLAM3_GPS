//
// Created by luk24515 on 14.12.25.
//
#include "../../include/GPS/GPSUtils.h"
#include <cmath>

namespace GPSUtils {

// WGS84 constants
static constexpr double a = 6378137.0;                 // semi-major axis
static constexpr double f = 1.0 / 298.257223563;
static constexpr double b = a * (1.0 - f);
static constexpr double e_sq = f * (2.0 - f);

// Origin in ECEF
static Eigen::Vector3d ecef_origin;
static double lat0_rad = 0.0;
static double lon0_rad = 0.0;
static bool origin_set = false;

static Eigen::Vector3d LLAtoECEF(double lat_rad,
                                 double lon_rad,
                                 double alt_m)
{
    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double sin_lon = std::sin(lon_rad);
    double cos_lon = std::cos(lon_rad);

    double N = a / std::sqrt(1.0 - e_sq * sin_lat * sin_lat);

    double x = (N + alt_m) * cos_lat * cos_lon;
    double y = (N + alt_m) * cos_lat * sin_lon;
    double z = (N * (1.0 - e_sq) + alt_m) * sin_lat;

    return Eigen::Vector3d(x, y, z);
}

void SetENUOrigin(double lat_deg, double lon_deg, double alt_m)
{
    lat0_rad = lat_deg * M_PI / 180.0;
    lon0_rad = lon_deg * M_PI / 180.0;
    ecef_origin = LLAtoECEF(lat0_rad, lon0_rad, alt_m);
    origin_set = true;
}

Eigen::Vector3d LatLonAltToENU(double lat_deg,
                              double lon_deg,
                              double alt_m)
{
    if (!origin_set)
        throw std::runtime_error("GPSUtils: ENU origin not set");

    double lat_rad = lat_deg * M_PI / 180.0;
    double lon_rad = lon_deg * M_PI / 180.0;

    Eigen::Vector3d ecef = LLAtoECEF(lat_rad, lon_rad, alt_m);
    Eigen::Vector3d d = ecef - ecef_origin;

    double sin_lat0 = std::sin(lat0_rad);
    double cos_lat0 = std::cos(lat0_rad);
    double sin_lon0 = std::sin(lon0_rad);
    double cos_lon0 = std::cos(lon0_rad);

    Eigen::Matrix3d R;
    R << -sin_lon0,            cos_lon0,           0,
         -sin_lat0*cos_lon0,  -sin_lat0*sin_lon0,  cos_lat0,
          cos_lat0*cos_lon0,   cos_lat0*sin_lon0,  sin_lat0;

    return R * d; // [E, N, U]
}

} // namespace GPSUtils
