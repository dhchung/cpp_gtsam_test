#include <iostream>
// #include "pointcloud_processing.h"
#include <Eigen/Dense>
#include "parameters.h"

#include "math.h"
#include "calculate_transformations.h"
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

// Regular headers
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include "state_plane.h"
#include "planar_factor.h"
#include "odom_factor.h"
#include "tools.h"
#include "detect_loop.h"
#include "calculate_transformations.h"


int main(int argc, char** argv){

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = 
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(10)<<init_noise_translation,
                                                                init_noise_translation,
                                                                init_noise_translation,
                                                                init_noise_angle,
                                                                init_noise_angle,
                                                                init_noise_angle,
                                                                measure_noise_normal,
                                                                measure_noise_normal,
                                                                measure_noise_normal,
                                                                measure_noise_distance).finished());

    gtsam::noiseModel::Diagonal::shared_ptr odomNoise = 
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<odom_noise_translation,
                                                               odom_noise_translation,
                                                               odom_noise_translation,
                                                               odom_noise_angle,
                                                               odom_noise_angle,
                                                               odom_noise_angle).finished());
    
    gtsam::noiseModel::Diagonal::shared_ptr measNoise = 
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(4)<<measure_noise_normal,
                                                               measure_noise_normal,
                                                               measure_noise_normal,
                                                               measure_noise_distance).finished());


    gtsamexample::StatePlane prior_factor = 
        gtsamexample::StatePlane(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0);

    gtsam::Values initials;

    int gtsam_idx = 0;
    gtsam::NonlinearFactorGraph graph;
    graph.add(gtsam::PriorFactor<gtsamexample::StatePlane>(gtsam_idx, prior_factor, priorNoise));
    initials.insert(gtsam_idx, prior_factor);

    // gtsam::Vector measurement = (gtsam::Vector(5)<<0,0,0,0,0).finished();
    gtsam::Vector6 measurement = gtsam::Vector6::Zero(6);
    measurement(0) = 1.0;
    measurement(1) = 1.0;
    measurement(2) = 1.0;
    measurement(3) = 0.0;
    measurement(4) = 0.0;
    measurement(5) = 2.0*M_PI/180.0f;

    graph.add(boost::make_shared<OdomFactor>(gtsam_idx, gtsam_idx+1, measurement, odomNoise));
    initials.insert(gtsam_idx+1, gtsamexample::StatePlane(2.2, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0));

    gtsam::Values results = gtsam::LevenbergMarquardtOptimizer(graph, initials).optimize();

    initials.print("Initials\n");
    results.print("Results\n");

}