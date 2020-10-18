#include <iostream>
#include "image_processing.h"
#include "altimeter_processing.h"

using namespace std;

int main(int argc, char** argv){

    ImageProcessing img_proc;
    AltimeterProcessing alt_proc;

    string image_dir = "/media/dongha/BLACK_PANTH/Dataset/170316_data/RectifiedImgs_Color";

    int data_num = 1388;
    alt_proc.load_altimeter(image_dir, data_num);
    
    for(int i = 1; i<data_num+1; ++i) {

        double cur_depth = alt_proc.depth[i-1];

        //Image pre-processing

        img_proc.load_image(image_dir, i);
        img_proc.apply_clahe(4.0);
        img_proc.show_image(img_proc.p_l_img, img_proc.p_r_img,1);
        // img_proc.show_image(10);

    }
    return 0;

    int k = 0;
    double asd = 0;
    k++;
    cout<<k<<endl;
}