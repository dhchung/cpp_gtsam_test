#include "detect_loop.h"

DetectLoop::DetectLoop(){}

DetectLoop::~DetectLoop(){}

void DetectLoop::find_loop_distance(gtsamexample::StatePlane & expected, gtsam::Values &graph,
                                    std::vector<int>* loop_candidate_idx,
                                    std::vector<float>* loop_candidate_dist){
    loop_candidate_dist->clear();
    loop_candidate_idx->clear();

    Eigen::Vector4f base_plane_eqn;
    Eigen::Vector3f base_mean_pt;

    get_values(expected, &base_plane_eqn, &base_mean_pt);

    for(int i=0; i<graph.size();++i){
        gtsamexample::StatePlane state = graph.at<gtsamexample::StatePlane>(i);
        Eigen::Vector4f quary_plane_eqn;
        Eigen::Vector3f quary_mean_pt;
        get_values(state, &quary_plane_eqn, &quary_mean_pt);
        float distance = calculate_distance(base_plane_eqn, base_mean_pt, quary_plane_eqn, quary_mean_pt);
        if(distance<1.0f){
            loop_candidate_idx->push_back(i);
            loop_candidate_dist->push_back(distance);
        }
    }

}

void DetectLoop::get_values(gtsamexample::StatePlane & state,
                            Eigen::Vector4f* global_plane_eqn,
                            Eigen::Vector3f* global_mean_pt){
    Eigen::Matrix4f T;
    c_trans.xyzrpy2t(state.x, state.y, state.z, state.roll, state.pitch, state.yaw, &T);
    Eigen::Vector4f Plane_eqn;
    Plane_eqn<<state.nx, state.ny, state.nz, state.d;
    Eigen::Matrix4f Eye_4 = Eigen::Matrix4f::Identity(4,4);
    Eigen::Vector3f Pt;
    Pt<<-state.d/state.nx, 0.0f, 0.0f;

    global_plane_eqn->operator=(c_trans.transform_plane(T, Plane_eqn, Eye_4));
    global_mean_pt->operator=(c_trans.transform_point(T, Pt));
}


float DetectLoop::calculate_distance(Eigen::Vector4f& base_plane_eqn,
                                     Eigen::Vector3f& base_mean_pt,
                                     Eigen::Vector4f& quary_plane_eqn,
                                     Eigen::Vector3f& quary_mean_pt){

    Eigen::Vector3f base_n = base_plane_eqn.segment(0,3);
    Eigen::Vector3f quary_n = quary_plane_eqn.segment(0,3);

    float t = -(base_plane_eqn(3) + base_n.transpose()*quary_mean_pt)/(base_n.transpose()*quary_n);
    Eigen::Vector3f quary_on_base = quary_mean_pt + t*quary_n;

    Eigen::Vector3f error = base_mean_pt - quary_on_base;

    float distance = sqrt(error.transpose()*error);
    return distance;
}