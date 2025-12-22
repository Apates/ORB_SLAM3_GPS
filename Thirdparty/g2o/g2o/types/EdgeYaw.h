//
// Created by luk24515 on 16.12.25.
//

#ifndef ORB_SLAM3_EDGEYAW_H
#define ORB_SLAM3_EDGEYAW_H
#pragma once

#include "types_six_dof_expmap.h"
#include "../core/base_unary_edge.h"

class EdgeYaw : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeYaw() {}

    void computeError() override
    {
        const g2o::VertexSE3Expmap* v =
            static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);

        // Get rotation
        Eigen::Matrix3d R = v->estimate().rotation().toRotationMatrix();

        // Extract yaw (Z axis rotation)
        double yaw = std::atan2(R(1,0), R(0,0));

        double error = yaw - _measurement;

        // Normalize angle to [-pi, pi]
        while (error > M_PI)  error -= 2.0 * M_PI;
        while (error < -M_PI) error += 2.0 * M_PI;

        _error[0] = error;
    }

    void linearizeOplus() override
    {
        // Numerical Jacobian is sufficient and safe
        _jacobianOplusXi.setZero();
        _jacobianOplusXi(0, 5) = 1.0;
    }

    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }
};

#endif //ORB_SLAM3_EDGEYAW_H