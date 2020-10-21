#pragma once
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/shapes.h>
#include <vector>
#include <Eigen/Dense>
#include "point_cloud.h"

class PointCloudProcessing{
public:
    void show_pointcloud(PointCloud & pt_cld);
};