#include <iostream>
// #include "pointcloud_processing.h"
#include <Eigen/Dense>
#include "parameters.h"

#include "math.h"
#include "calculate_transformations.h"
#include <vector>

#include <random>
#include <iterator>

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
#include "opengl_point_processing.h"

OpenglPointProcessing ogl_pt_processing("3D Point Cloud (non-sequential)");

int main(int argc, char** argv){
    ogl_pt_processing.init_opengl();

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

    std::default_random_engine generator;
    const float stddev = 1;
    const float mean = 0.0f;
    std::normal_distribution<float> dist(mean, stddev);


    std::vector<gtsam::Vector6> odometry;
    std::vector<gtsam::Vector4> measurement;
    for(int i = 0; i < 2; ++i){
        //moves in y direction
        gtsam::Vector6 odom;
        // odom<<0.0, 1.0, 0.0, 0.0, 0.0, dist(generator)*1*M_PI/180.0;
        odom = gtsam::Vector6::Zero(6);
        odometry.push_back(odom);
        gtsam::Vector4 meas;
        meas<<-1.0, 0.0, 0.0, dist(generator);
        measurement.push_back(meas);

        std::cout<<meas.transpose()<<std::endl;
    }

    for(int i = 0; i < 2; ++i){
        graph.add(boost::make_shared<OdomFactor>(gtsam_idx, gtsam_idx+1, odometry[i], odomNoise));
        graph.add(boost::make_shared<PlanarFactor>(gtsam_idx, gtsam_idx+1, measurement[i], measNoise));
        initials.insert(gtsam_idx+1, gtsamexample::StatePlane(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0));
        ++gtsam_idx;
    }


    gtsam::Values results = gtsam::LevenbergMarquardtOptimizer(graph, initials).optimize();

    initials.print("Initials\n");
    results.print("Results\n");
    ogl_pt_processing.draw_surfels(results);

}