#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/shapes.h>
#include <vector>


class PointCloudProcessing{
public:
    void show_pointcloud(std::vector<std::vector<float>> & pt_cld);
};