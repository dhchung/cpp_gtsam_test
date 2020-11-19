#include "pt_cloud.h"


PointCloud::PointCloud(){

    rel_state_exist = false;
    gt_state_exist = false;

    rel_state.clear();
    state.clear();

}
PointCloud::~PointCloud(){
    
}

void PointCloud::input_rel_state(std::vector<float> & relative_state){
    rel_state_exist = true;
    rel_state = relative_state;
}


void PointCloud::input_gt_state(std::vector<float> & groundtruth_state){
    gt_state_exist = true;
    state = groundtruth_state;
}


void PointCloud::change_state(std::vector<float> & changed_state){
    state = changed_state;
}

void PointCloud::estimate_plane_model(){
    //plane_model;

    Eigen::Vector3f mean = point_cloud.rowwise().mean();
    const Eigen::Matrix3Xf points_centered = point_cloud.colwise() - mean;

    int setting = Eigen::ComputeThinU | Eigen::ComputeThinV;
    Eigen::JacobiSVD<Eigen::Matrix3Xf> svd = points_centered.jacobiSvd(setting);

    Eigen::Vector3f normal = svd.matrixU().col(2);

    float d = -normal.transpose()*mean;

    if(d < 0){
        d = -d;
        normal = -normal;
    }

    
    plane_model.segment(0,3) = normal;
    plane_model(3) = d;
}