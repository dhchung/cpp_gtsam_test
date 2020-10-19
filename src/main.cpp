#include <iostream>
#include "image_processing.h"
#include "altimeter_processing.h"
#include "pointcloud_processing.h"

using namespace std;

int main(int argc, char** argv){

    ImageProcessing img_proc;
    AltimeterProcessing alt_proc;

    PointCloudProcessing pt_cld_processing;

    string image_dir = "/media/dongha/BLACK_PANTH/Dataset/170316_data/RectifiedImgs_Color";

    int data_num = 1388;
    alt_proc.load_altimeter(image_dir, data_num);
    float depth_err = 300;
    
    for(int i = 1; i<data_num+1; ++i) {

        float cur_depth = alt_proc.depth[i-1];
        std::cout<<"depth: "<<cur_depth<<std::endl;
        //Image pre-processing

        img_proc.load_image(image_dir, i);
        img_proc.apply_clahe(4.0);
        img_proc.show_image(img_proc.p_l_img, img_proc.p_r_img,1);
        img_proc.match_stereo(cur_depth, depth_err);
        img_proc.get_3d_points();
        // img_proc.show_image(10);


        std::vector<std::vector<float>> point_3d;
        point_3d.resize(img_proc.surf.point_3d.size());
        for(int i=0; i<point_3d.size(); ++i) {
            std::vector<float> point{img_proc.surf.point_3d[i].loc_3d_x,
                                     img_proc.surf.point_3d[i].loc_3d_y,
                                     img_proc.surf.point_3d[i].loc_3d_z};
            point_3d[i] = point;
        }
        pt_cld_processing.show_pointcloud(point_3d);
        
    }
    return 0;
}