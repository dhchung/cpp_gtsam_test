#include "point_cloud.h"


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