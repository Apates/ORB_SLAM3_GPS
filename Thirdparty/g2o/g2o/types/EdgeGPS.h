#pragma once
#include "types_six_dof_expmap.h" // VertexSE3Expmap
#include "../core/base_unary_edge.h"
#include <Eigen/Core>

using namespace g2o;

class EdgeGPS : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeGPS() = default;

    void computeError() override {
        const VertexSE3Expmap* v = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        // Extract translation from vertex estimate
        Eigen::Vector3d t = v->estimate().translation();
        _error = t - _measurement; // predicted - measured (we could do measured - predicted; must be consistent)
    }

    // Optional: robustify or clamp large errors if you want
    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }
};
