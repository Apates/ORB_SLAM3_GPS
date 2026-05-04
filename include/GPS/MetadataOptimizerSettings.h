//
// Created by luk24515 on 28.04.26.
//

#ifndef ORB_SLAM3_METADATAOPTIMIZERSETTINGS_H
#define ORB_SLAM3_METADATAOPTIMIZERSETTINGS_H


struct MetadataOptimizerSettings {
public:
    float GPSWeight = 0.04f; // Relative weight of GPS error term in optimization
    float GPSHuberDelta = 1.0f; // Huber loss delta for GPS error term
    bool GlobalGPS = false; // Whether to apply GPS constraints globally
    bool PeriodicGlobalBA = false; // Whether to apply Global BA periodically (every 20 Keyframes)
    bool LocalGPS = true; // Whether to apply GPS constraints to local keyframes
    bool TransformToENU = false; // Whether to transform SLAM coordinates to ENU
};


#endif //ORB_SLAM3_METADATAOPTIMIZERSETTINGS_H