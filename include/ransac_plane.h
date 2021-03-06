#pragma once
#include <vector>
#include <cstdlib>
#include <iostream>
#include <Eigen/Dense>
#include "pt_cloud.h"

class RANSACPlane{
public:
    RANSACPlane();
    ~RANSACPlane();

    void select_index(int & data_num, std::vector<int> *indices);

    void perform_ransac_plane(PointCloud & input,
                              PointCloud * output);
    void get_indices(Eigen::VectorXf & distance, 
                     std::vector<int> * idx, 
                     float & distnace_threshold);

    void estimate_plane(PointCloud * input);
    void estimate_plane_with_uncertainty(PointCloud * input);
};