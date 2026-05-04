//
// Created by luk24515 on 14.12.25.
//
#include "../../include/GPS/GPSUtils.h"
#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "MapPoint.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace GPSUtils {
    // WGS84 constants
    static constexpr double a = 6378137.0; // semi-major axis
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
                                     double alt_m) {
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

    void SetENUOrigin(double lat_deg, double lon_deg, double alt_m) {
        lat0_rad = lat_deg * M_PI / 180.0;
        lon0_rad = lon_deg * M_PI / 180.0;
        ecef_origin = LLAtoECEF(lat0_rad, lon0_rad, alt_m);
        origin_set = true;
    }

    Eigen::Vector3d LatLonAltToENU(double lat_deg,
                                   double lon_deg,
                                   double alt_m) {
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
        R << -sin_lon0, cos_lon0, 0,
                -sin_lat0 * cos_lon0, -sin_lat0 * sin_lon0, cos_lat0,
                cos_lat0 * cos_lon0, cos_lat0 * sin_lon0, sin_lat0;

        return R * d; // [E, N, U]
    }

    /**
 * Calculates the Sim3 transformation (s, R, t) that aligns SLAM points to GPS points.
 * Minimizes the error: || P_gps - (s * R * P_slam + t) ||^2
 */
    g2o::Sim3 CalculateSim3Alignment(const std::vector<Eigen::Vector3f> &vP_slam,
                                     const std::vector<Eigen::Vector3d> &vP_gps,
                                     bool* success) {
        if (vP_slam.size() != vP_gps.size() || vP_slam.size() < 3) {
            std::cerr << "Error: Need at least 3 matched points for Umeyama alignment." << std::endl;
            *success = false;
            return g2o::Sim3(); // Return identity
        }

        size_t N = vP_slam.size();

        // Convert std::vectors to Eigen::Matrix (3 x N)
        // Eigen::umeyama requires points as columns in a matrix
        Eigen::MatrixXd Mat_slam(3, N);
        Eigen::MatrixXd Mat_gps(3, N);

        for (size_t i = 0; i < N; ++i) {
            Mat_slam.col(i) = vP_slam[i].cast<double>();
            Mat_gps.col(i) = vP_gps[i];
        }

        // Compute the Transformation Matrix
        Eigen::Matrix4d T_out = Eigen::umeyama(Mat_slam, Mat_gps, true);

        if (T_out.hasNaN()) {
            std::cerr << "Error: Umeyama failed (NaN result)." << std::endl;
            *success = false;
            return g2o::Sim3();
        }

        // Extract Rotation, Translation, and Scale from the 4x4 Matrix
        Eigen::Matrix3d sR = T_out.block<3, 3>(0, 0);
        Eigen::Vector3d t = T_out.block<3, 1>(0, 3);

        // Calculate scale from the volume determinant
        double scale = std::pow(sR.determinant(), 1.0 / 3.0);

        // If scale is negative, it implies a reflection (coordinate system flip), which is invalid for SLAM
        if (scale < 0) {
            std::cerr << "Warning: Umeyama detected reflection. Forcing positive scale." << std::endl;
            scale = -scale;
        }

        // Recover pure Rotation matrix
        Eigen::Matrix3d R = sR / scale;

        // Orthogonalize R to fix any numerical noise (SVD cleanup)
        // This ensures R is a valid rotation matrix
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R_clean = svd.matrixU() * svd.matrixV().transpose();

        // Handle reflection in rotation recovery
        if (R_clean.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1; // Flip one axis
            R_clean = svd.matrixU() * V.transpose();
        }

        // g2o::Sim3 stores the transform from Local -> World
        *success = true;
        return g2o::Sim3(R_clean, t, scale);
    }

    void TransformMapToGlobal(const g2o::Sim3 &T_final, std::vector<ORB_SLAM3::MapPoint *> &vpMPs) {
        // T_final is the Sim3 from Local SLAM -> Global ENU

        std::cout << "Transforming map points..." << std::endl;
        for (auto pMP: vpMPs) {
            if (!pMP || pMP->isBad()) continue;

            // Get current Local Position (Vector3f)
            Eigen::Vector3f P_local = pMP->GetWorldPos();

            // Transform to Global
            Eigen::Vector3d P_global = T_final.map(P_local.cast<double>());

            // Update MapPoint Position
            pMP->SetWorldPos(P_global.cast<float>());

            // This recomputes the normal vector and depth range based on
            // the new relative geometry of the KeyFrames that see this point.
            pMP->UpdateNormalAndDepth();
        }
    }

    void TransformKeyframesToGlobal(const g2o::Sim3 &T_init, std::vector<ORB_SLAM3::KeyFrame *> &vpKFs) {
        for (auto *pKF: vpKFs) {
            Sophus::SE3f T_cw_f = pKF->GetPose();

            // Convert to double for the Global Math
            Sophus::SE3d T_cw_local = T_cw_f.cast<double>();

            Sophus::SE3d T_wc_local = T_cw_local.inverse();

            // Retrieve optimized Sim3 transform
            Eigen::Matrix3d R_final = T_init.rotation().toRotationMatrix();
            Eigen::Vector3d t_final = T_init.translation();
            double s_final = T_init.scale();

            // Apply Sim3 to the translation and Rotation
            Eigen::Vector3d P_global = s_final * (R_final * T_wc_local.translation()) + t_final;
            Eigen::Matrix3d R_global = R_final * T_wc_local.rotationMatrix();

            Sophus::SE3d T_wc_global(R_global, P_global);

            pKF->SetPose(T_wc_global.inverse().cast<float>());
        }
    }
}
