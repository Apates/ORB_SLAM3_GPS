//
// Created by luk24515 on 14.12.25.
//

#ifndef ORB_SLAM3_GPSUTILS_H
#define ORB_SLAM3_GPSUTILS_H
#pragma once
#include <Eigen/Core>

namespace GPSUtils {

    // Set the ENU origin (call once at startup)
    void SetENUOrigin(double lat_deg, double lon_deg, double alt_m);

    // Convert lat/lon/alt to ENU (meters)
    Eigen::Vector3d LatLonAltToENU(double lat_deg,
                                  double lon_deg,
                                  double alt_m);

} // namespace GPSUtils

#endif //ORB_SLAM3_GPSUTILS_H