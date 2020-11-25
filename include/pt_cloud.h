#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <vector>

class PointCloud{
public:


    bool plane_uncertainty;

    std::vector<float> state;
    std::vector<float> dr_state;
    std::vector<float> rel_state;

    PointCloud();
    ~PointCloud();
    Eigen::Matrix3Xf point_cloud;
    Eigen::Matrix3Xi point_color;
    Eigen::MatrixXf point_des;
    Eigen::VectorXf point_size;

    Eigen::Vector4f plane_model;
    

    void input_rel_state(std::vector<float> & relative_state);
    void input_dr_state(std::vector<float> & dr_state_i);
    void change_state(std::vector<float> & changed_state);

    void estimate_plane_model();
};