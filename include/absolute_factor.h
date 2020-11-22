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
#include "calculate_transformations.h"

using namespace gtsam;
using namespace gtsamexample;
class AbsoluteFactor: public NoiseModelFactor1<StatePlane>{

private:
    Vector3 measured_;

public:

    AbsoluteFactor(Key key1, 
                 const Vector3& measured, const SharedNoiseModel& model = nullptr);

    Vector evaluateError(const StatePlane& sp1,
                         boost::optional<Matrix&> H = boost::none) const;

};