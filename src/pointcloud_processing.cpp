#include "pointcloud_processing.h"

void PointCloudProcessing::show_pointcloud(std::vector<std::vector<float>> & pt_cld){

    pcl::PointCloud<pcl::PointXYZ> cloud;

    std::cout<<pt_cld.size()<<std::endl;

    cloud.width = pt_cld.size();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width*cloud.height);

    std::cout<<"size : "<<cloud.points.size()<<std::endl;

    for(int i=0; i<cloud.points.size(); ++i) {
        // std::cout<<i<<".x: "<<pt_cld[i][0]<<std::endl;
        // std::cout<<i<<".y: "<<pt_cld[i][1]<<std::endl;
        // std::cout<<i<<".z: "<<pt_cld[i][2]<<std::endl;

        cloud.points[i].x = pt_cld[i][0]/1000.0;
        cloud.points[i].y = pt_cld[i][1]/1000.0;
        cloud.points[i].z = pt_cld[i][2]/1000.0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud2 = cloud;
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");



    viewer.showCloud(cloud2);
    // while (!viewer.wasStopped ())
    // {
    // }    
    
    
}



// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
// *cloudPTR = createPointCloud(nodeList);