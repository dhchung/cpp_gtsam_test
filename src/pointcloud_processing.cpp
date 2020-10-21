#include "pointcloud_processing.h"

void PointCloudProcessing::show_pointcloud(PointCloud & pt_cld){

    pcl::PointCloud<pcl::PointXYZ> cloud;


    cloud.width = pt_cld.point_cloud.cols();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width*cloud.height);

    std::cout<<pt_cld.point_cloud<<std::endl;

    for(int i=0; i<cloud.points.size(); ++i) {
        cloud.points[i].x = pt_cld.point_cloud(0,i)/1000.0;
        cloud.points[i].y = pt_cld.point_cloud(1,i)/1000.0;
        cloud.points[i].z = pt_cld.point_cloud(2,i)/1000.0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud2 = cloud;
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

    viewer.showCloud(cloud2);
    while (!viewer.wasStopped ())
    {
    }    
    
    
}