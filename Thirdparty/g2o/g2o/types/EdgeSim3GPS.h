//
// Created by luk24515 on 22.12.25.
//
#pragma once

#include "../core/base_unary_edge.h"
#include <Eigen/Core>

#include "types_seven_dof_expmap.h"

#ifndef ORB_SLAM3_EDGESIM3GPS_H
#define ORB_SLAM3_EDGESIM3GPS_H

class EdgeSim3GPS : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap, g2o::VertexSim3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3GPS() {}

    void computeError() override {
        // Vertex 0: The KeyFrame Pose (SE3)
        const g2o::VertexSE3Expmap* vKF = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        // Vertex 1: The Global Alignment (Sim3)
        const g2o::VertexSim3Expmap* vAlign = static_cast<const g2o::VertexSim3Expmap*>(_vertices[1]);

        // 1. Get Camera Center in Local SLAM Frame
        // T_cw is stored, so we need inverse translation for center
        Eigen::Vector3d P_local = vKF->estimate().inverse().translation();

        // 2. Transform Local -> Global using the Alignment Vertex
        // Sim3 map function: (s * R * p) + t
        Eigen::Vector3d P_global_est = vAlign->estimate().map(P_local);

        // 3. Error is difference against real GPS measurement
        _error = P_global_est - _measurement;
    }

    // Jacobians are tricky here, but essential for stability.
    // If you don't implement linearizeOplus, g2o uses numeric differentiation (slower).
    // For now, numeric is fine to test functionality.


    // 3. READ method (Must be present for the Factory/Macros)
    virtual bool read(std::istream& is) override {
        for (int i=0; i<3; ++i) is >> _measurement[i];
        for (int i=0; i<3; ++i)
            for (int j=i; j<3; ++j) {
                is >> information()(i,j);
                if (i!=j) information()(j,i) = information()(i,j);
            }
        return true;
    }

    // 4. WRITE method (Must be present for the Factory/Macros)
    virtual bool write(std::ostream& os) const override {
        for (int i=0; i<3; ++i) os << measurement()[i] << " ";
        for (int i=0; i<3; ++i)
            for (int j=i; j<3; ++j) os << " " << information()(i,j);
        return os.good();
    }
};

#endif //ORB_SLAM3_EDGESIM3GPS_H