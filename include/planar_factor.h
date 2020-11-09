#pragma once
#include <gtsam/base/Testable.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/NoiseModel.h>
#include "state_plane.h"
#include "math.h"

using namespace gtsam;
using namespace gtsamexample;
class PlanarFactor: public NoiseModelFactor2<StatePlane, StatePlane>{

private:
    Vector4 measured_;

public:

    PlanarFactor(Key key1, Key key2, 
                 const Vector4& measured, const SharedNoiseModel& model = nullptr);

    Vector evaluateError(const StatePlane& sp1, const StatePlane & sp2,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const;


    Matrix rollMatrix(float roll, bool jacobian = false) const;
    Matrix pitchMatrix(float pitch, bool jacobian = false) const;
    Matrix yawMatrix(float yaw, bool jacobian = false) const;
    Matrix rotationMatrix(float roll, float pitch, float yaw) const;
    Matrix rollJacobiMatrix(float roll, float pitch, float yaw) const;
    Matrix pitchJacobiMatrix(float roll, float pitch, float yaw) const;
    Matrix yawJacobiMatrix(float roll, float pitch, float yaw) const;

};