#include <iostream>
#include "data_path.h"
#include "image_processing.h"
#include "altimeter_processing.h"
// #include "pointcloud_processing.h"
#include "ransac_plane.h"
#include <thread>
#include <Eigen/Dense>
#include "point_cloud.h"
#include "opengl_point_processing.h"
// #include "slam.h"
#include "dr_processing.h"
#include "parameters.h"


using std::thread;
using namespace std;

ImageProcessing img_proc;
AltimeterProcessing alt_proc;
// PointCloudProcessing pt_cld_processing;
RANSACPlane ransac_plane;
PointCloud pt_cld;
OpenglPointProcessing ogl_pt_processing("3D Point Cloud (non-sequential)");

DrProcessing dr_processing;

int main(int argc, char** argv){

    ogl_pt_processing.init_opengl();

    string image_dir = DATA_PATH;

    int data_num = 1388;
    alt_proc.load_altimeter(image_dir, data_num);

    dr_processing.load_dr(image_dir, data_num);


    std::vector<PointCloud> global_cloud;

    for(int i = 120; i<data_num-6; ++i) {

        std::vector<float> cur_dr_state = dr_processing.nav_data[i];
        std::vector<float> rel_dr_prev_state = dr_processing.rel_nav_data[i-1];

        float cur_depth = alt_proc.depth[i-1];
        //Image pre-processing

        img_proc.load_image(image_dir, i);
        img_proc.apply_clahe(4.0);
        img_proc.show_image(img_proc.l_img, img_proc.r_img,1);
        img_proc.match_stereo(cur_depth, &pt_cld);


        PointCloud ransac_point_3d;
        ransac_plane.perform_ransac_plane(pt_cld, &ransac_point_3d);
        ransac_point_3d.input_gt_state(cur_dr_state);
        ransac_point_3d.input_rel_state(rel_dr_prev_state);

        global_cloud.push_back(ransac_point_3d);

        ogl_pt_processing.plot_global_points(global_cloud, ransac_point_3d.state, i);
    }
    ogl_pt_processing.draw_point_global(global_cloud, 3.0f);
    ogl_pt_processing.terminate();
    return 0;
}


