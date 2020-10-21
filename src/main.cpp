#include <iostream>
#include "data_path.h"
#include "image_processing.h"
#include "altimeter_processing.h"
#include "pointcloud_processing.h"
#include "ransac_plane.h"
#include <thread>
#include <Eigen/Dense>
#include "point_cloud.h"


using std::thread;
using namespace std;

ImageProcessing img_proc;
AltimeterProcessing alt_proc;
PointCloudProcessing pt_cld_processing;
RANSACPlane ransac_plane;
PointCloud pt_cld;

// void f1(string & img_dir, int & img_no, float & cur_depth, float &depth_err){
//     img_proc.load_image(img_dir, img_no);
//     img_proc.apply_clahe(4.0);
//     img_proc.show_image(img_proc.p_l_img, img_proc.p_r_img,1);
//     img_proc.match_stereo(cur_depth, depth_err);
//     img_proc.get_3d_points();
// }

int main(int argc, char** argv){
    string image_dir = DATA_PATH;

    int data_num = 1388;
    alt_proc.load_altimeter(image_dir, data_num);
    float depth_err = 300;
    
    for(int i = 120; i<data_num+1; ++i) {

        float cur_depth = alt_proc.depth[i-1];
        //Image pre-processing

        img_proc.load_image(image_dir, i);
        img_proc.apply_clahe(4.0);
        img_proc.show_image(img_proc.p_l_img, img_proc.p_r_img,1);
        img_proc.match_stereo(cur_depth, depth_err, &pt_cld);
        // thread t1(f1, image_dir, i, cur_depth, depth_err);
        // f1(image_dir, i, cur_depth, depth_err);
        // t1.join();
        PointCloud ransac_point_3d;
        ransac_plane.perform_ransac_plane(pt_cld, &ransac_point_3d);

        pt_cld_processing.show_pointcloud(pt_cld);

        pt_cld_processing.show_pointcloud(ransac_point_3d);

        
    }
    return 0;
}

