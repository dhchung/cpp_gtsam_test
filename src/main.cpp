#include <iostream>
#include "data_path.h"
#include "image_processing.h"
#include "altimeter_processing.h"
#include "ransac_plane.h"
#include <Eigen/Dense>
#include "pt_cloud.h"
#include "opengl_point_processing.h"
#include "dr_processing.h"
#include "parameters.h"

#include "math.h"
#include "calculate_transformations.h"
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

// Regular headers
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include "state_plane.h"
#include "planar_factor.h"
#include "odom_factor.h"

#include "tools.h"

#include "detect_loop.h"

#include "absolute_factor.h"

using namespace std;

using namespace gtsam;
using namespace gtsamexample;

ImageProcessing img_proc;
AltimeterProcessing alt_proc;
// PointCloudProcessing pt_cld_processing;
RANSACPlane ransac_plane;
PointCloud pt_cld;
OpenglPointProcessing ogl_pt_processing("3D Point Cloud (non-sequential)");

DrProcessing dr_processing;

Tools tools;

DetectLoop detect_loop;

int main(int argc, char** argv){


    //OpenGL Initialization
    ogl_pt_processing.init_opengl();

    string image_dir = DATA_PATH;

    int data_num = 1388;
    alt_proc.load_altimeter(image_dir, data_num);

    dr_processing.load_dr(image_dir, data_num);

    std::vector<PointCloud> global_cloud;

    //gtsam
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initials;
    int gtsam_idx = 0;


    gtsam::noiseModel::Diagonal::shared_ptr absoluteNoise = 
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(absolute_noise_translation,
                                                           absolute_noise_angle,
                                                           absolute_noise_angle));

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = 
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(10)<<init_noise_translation,
                                                                init_noise_translation,
                                                                init_noise_translation,
                                                                init_noise_angle,
                                                                init_noise_angle,
                                                                init_noise_angle,
                                                                measure_noise_normal,
                                                                measure_noise_normal,
                                                                measure_noise_normal,
                                                                measure_noise_distance).finished());

    gtsam::noiseModel::Diagonal::shared_ptr odomNoise = 
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<odom_noise_translation,
                                                               odom_noise_translation,
                                                               odom_noise_translation,
                                                               odom_noise_angle,
                                                               odom_noise_angle,
                                                               odom_noise_angle).finished());
    
    gtsam::noiseModel::Diagonal::shared_ptr measNoise = 
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(4)<<measure_noise_normal,
                                                               measure_noise_normal,
                                                               measure_noise_normal,
                                                               measure_noise_distance).finished());

    int initial_data_no = 120;
    int final_data_no = data_num-6;
    // final_data_no = 300;

    Values inloop_result;

    for(int i = initial_data_no; i<final_data_no; ++i) {

        std::cout<<"data : "<<i<<" out of "<<final_data_no<<std::endl;

        std::vector<float> cur_dr_state = dr_processing.nav_data[i];
        std::vector<float> rel_dr_prev_state = dr_processing.rel_nav_data[i-1];

        float cur_depth = alt_proc.depth[i-1];
        //Image pre-processing

        img_proc.load_image(image_dir, i);
        img_proc.apply_clahe(4.0);
        // img_proc.show_image(img_proc.l_img, img_proc.r_img,1);
        img_proc.match_stereo(cur_depth, &pt_cld);

        PointCloud ransac_point_3d;
        ransac_plane.perform_ransac_plane(pt_cld, &ransac_point_3d);

        ransac_point_3d.estimate_plane_model();
        ransac_point_3d.input_gt_state(cur_dr_state);
        ransac_point_3d.input_rel_state(rel_dr_prev_state);


        if(i==initial_data_no){
            //let's start from nav_data[i] and its measurements
            gtsamexample::StatePlane init_sp_state = 
                gtsamexample::StatePlane(cur_dr_state[0],
                                         cur_dr_state[1],
                                         cur_dr_state[2],
                                         cur_dr_state[3],
                                         cur_dr_state[4],
                                         cur_dr_state[5],
                                         ransac_point_3d.plane_model(0),
                                         ransac_point_3d.plane_model(1),
                                         ransac_point_3d.plane_model(2),
                                         ransac_point_3d.plane_model(3));

            graph.add(gtsam::PriorFactor<gtsamexample::StatePlane>(gtsam_idx, init_sp_state, priorNoise));
            initials.insert(gtsam_idx, init_sp_state);
            ogl_pt_processing.insertImages(img_proc.l_img);
            global_cloud.push_back(ransac_point_3d);
            inloop_result = LevenbergMarquardtOptimizer(graph, initials).optimize();
            
        }else{

            gtsamexample::StatePlane cur_sp_state = 
                gtsamexample::StatePlane(cur_dr_state[0],
                                         cur_dr_state[1],
                                         cur_dr_state[2],
                                         cur_dr_state[3],
                                         cur_dr_state[4],
                                         cur_dr_state[5],
                                         ransac_point_3d.plane_model(0),
                                         ransac_point_3d.plane_model(1),
                                         ransac_point_3d.plane_model(2),
                                         ransac_point_3d.plane_model(3));

            std::vector<int> loop_candidates;
            std::vector<float> distance;

            gtsam::Vector4 measurement;
            measurement(0) = ransac_point_3d.plane_model(0);
            measurement(1) = ransac_point_3d.plane_model(1);
            measurement(2) = ransac_point_3d.plane_model(2);
            measurement(3) = ransac_point_3d.plane_model(3);

            // detect_loop.find_loop_distance(cur_sp_state, inloop_result, &loop_candidates, &distance);

            // for(int loop_id:loop_candidates){
            //     std::cout<<loop_id<<", ";
            // }
            // std::cout<<std::endl;


            // if(!loop_candidates.empty()){
            //     for(int id = 0; id<loop_candidates.size(); ++id){
            //         int candid_id = loop_candidates[id];
            //         double dist = (double)distance[id];

            //         double meas_noise_n = measure_noise_normal + exp(dist/5)-1.0;
            //         double meas_noise_d = measure_noise_normal + exp(dist)-1.0;

            //         gtsam::noiseModel::Diagonal::shared_ptr measNoise_d = 
            //             gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(4)<<meas_noise_n,
            //                                                                    meas_noise_n,
            //                                                                    meas_noise_n,
            //                                                                    meas_noise_d).finished());

            //         graph.add(boost::make_shared<PlanarFactor>(candid_id, gtsam_idx+1, measurement, measNoise_d));
            //     }
            // }



            gtsam::Vector6 rel_odom = tools.turnVectorToGTSAMVector(rel_dr_prev_state);
            graph.add(boost::make_shared<OdomFactor>(gtsam_idx, gtsam_idx+1, rel_odom, odomNoise));

            gtsam::Vector3 absolute_measure(cur_dr_state[2], cur_dr_state[3], cur_dr_state[4]);

            graph.add(boost::make_shared<AbsoluteFactor>(gtsam_idx+1, absolute_measure, absoluteNoise));

            // StatePlane prev_state = initials.at<StatePlane>

            graph.add(boost::make_shared<PlanarFactor>(gtsam_idx, gtsam_idx+1, measurement, measNoise));
            initials.insert(gtsam_idx+1, cur_sp_state);

            global_cloud.push_back(ransac_point_3d);
            
            // inloop_result = LevenbergMarquardtOptimizer(graph, initials).optimize();

            // for(int j = 0; j < inloop_result.size(); ++j){
            //     StatePlane optimized_result = inloop_result.at<StatePlane>(j);
            //     std::vector<float> optimized_state;
            //     optimized_state.push_back(optimized_result.x);
            //     optimized_state.push_back(optimized_result.y);
            //     optimized_state.push_back(optimized_result.z);
            //     optimized_state.push_back(optimized_result.roll);
            //     optimized_state.push_back(optimized_result.pitch);
            //     optimized_state.push_back(optimized_result.yaw);
            //     global_cloud[j].change_state(optimized_state);

            // }




            ++gtsam_idx;
            ogl_pt_processing.insertImages(img_proc.l_img);

        }

        ogl_pt_processing.plot_global_points(global_cloud, ransac_point_3d.state, i);
    }

    gtsam::Values results = LevenbergMarquardtOptimizer(graph, initials).optimize();
    // results.print("Final Result:\n");
    
    for(int i = 0; i < results.size(); ++i){
        StatePlane optimized_result = results.at<StatePlane>(i);
        std::vector<float> optimized_state;
        optimized_state.push_back(optimized_result.x);
        optimized_state.push_back(optimized_result.y);
        optimized_state.push_back(optimized_result.z);
        optimized_state.push_back(optimized_result.roll);
        optimized_state.push_back(optimized_result.pitch);
        optimized_state.push_back(optimized_result.yaw);

        global_cloud[i].change_state(optimized_state);
    }

    tools.evaluateError(initials, results);

    // ogl_pt_processing.draw_plane_global(results);
    // ogl_pt_processing.draw_plane_global_wo_texture(results);
    // ogl_pt_processing.draw_surfels(results);
    ogl_pt_processing.draw_point_global(global_cloud, 3.0f);
    ogl_pt_processing.terminate();
    return 0;
}


