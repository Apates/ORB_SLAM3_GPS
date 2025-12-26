//
// Created by luk24515 on 14.12.25.
//

#ifndef ORB_SLAM3_GPSUTILS_H
#define ORB_SLAM3_GPSUTILS_H
#pragma once
#include <Eigen/Core>
#include <vector>

#include "MapPoint.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace GPSUtils {

    // Set the ENU origin (call once at startup)
    void SetENUOrigin(double lat_deg, double lon_deg, double alt_m);

    // Convert lat/lon/alt to ENU (meters)
    Eigen::Vector3d LatLonAltToENU(double lat_deg,
                                  double lon_deg,
                                  double alt_m);
    g2o::Sim3 CalculateSim3Alignment(const std::vector<Eigen::Vector3f>& vP_slam,
                                 const std::vector<Eigen::Vector3d>& vP_gps,
                                 bool* success);

    void TransformMapToGlobal(const g2o::Sim3& T_final, std::vector<ORB_SLAM3::MapPoint*>& vpMPs);

} // namespace GPSUtils

#endif //ORB_SLAM3_GPSUTILS_H