//
// Created by luk24515 on 26.12.25.
//
#pragma once

#include "../core/base_unary_edge.h"
#include <Eigen/Core>

#include "types_seven_dof_expmap.h"
#ifndef ORB_SLAM3_EDGEGPSUNARY_H
#define ORB_SLAM3_EDGEGPSUNARY_H
class EdgeGPSUnary : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeGPSUnary() {}

    void computeError() override {
        const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        // Camera Center in SLAM frame
        Eigen::Vector3d p_slam = v->estimate().inverse().translation();
        // Error is difference between SLAM pos and the GPS-converted-to-SLAM pos
        _error = p_slam - _measurement;
    }

    virtual bool read(std::istream& is) { return true; }
    virtual bool write(std::ostream& os) const { return true; }
};
#endif //ORB_SLAM3_EDGEGPSUNARY_H