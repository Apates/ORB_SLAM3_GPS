//
// Created by luk24515 on 14.12.25.
//

#ifndef ORB_SLAM3_GPSMANAGER_H
#define ORB_SLAM3_GPSMANAGER_H
#pragma once

#include <Eigen/Core>
#include <vector>
#include <string>

struct GPSMeasurement
{
    double timestamp;            // seconds (relative or UNIX, must match Frame timestamp base)
    Eigen::Vector3d enu;         // meters (ENU)
};

class GPSManager
{
public:
    GPSManager() = default;

    // Load and parse SRT file (returns false on failure)
    bool LoadFromSRT(const std::string& path);

    // Nearest-neighbor lookup
    // Returns true if a GPS measurement within max_dt seconds exists
    bool GetENUAtTime(double t,
                      Eigen::Vector3d& enu,
                      double max_dt = 0.1) const;

private:
    std::vector<GPSMeasurement> mMeasurements;

    // helpers
    static double ParseTimestampToSeconds(const std::string& line);
    static bool ExtractDouble(const std::string& line,
                              const std::string& key,
                              double& value);
};


#endif //ORB_SLAM3_GPSMANAGER_H