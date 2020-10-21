#pragma once
#include <iostream>
#include <Eigen/Dense>

class PointCloud{
public:
    PointCloud();
    ~PointCloud();
    Eigen::Matrix3Xf point_cloud;
    Eigen::Matrix3Xi point_color;
    Eigen::MatrixXf point_des;
    Eigen::VectorXf point_size;
};