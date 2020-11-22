#pragma once
#include <gtsam/nonlinear/Values.h>
#include <vector>
#include "state_plane.h"
#include "calculate_transformations.h"
#include "Eigen/Dense"

class DetectLoop{
public:
    DetectLoop();
    ~DetectLoop();

    CalTransform c_trans;

    void find_loop_distance(gtsamexample::StatePlane & expected, gtsam::Values &graph,
                            std::vector<int>* loop_candidate_idx,
                            std::vector<float>* loop_candidate_dist);

    void get_values(gtsamexample::StatePlane & state,
                    Eigen::Vector4f* global_plane_eqn,
                    Eigen::Vector3f* global_mean_pt);


    float calculate_distance(Eigen::Vector4f& base_plane_eqn,
                              Eigen::Vector3f& base_mean_pt,
                              Eigen::Vector4f& quary_plane_eqn,
                              Eigen::Vector3f& quary_mean_pt);

};