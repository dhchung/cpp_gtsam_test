#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image_processing.h"

using namespace std;

int main(int argc, char** argv){

    ImageProcessing img_proc;


    string image_dir = "/media/dongha/BLACK_PANTH/Dataset/170316_data/RectifiedImgs_Color";
    
    for(int i = 1; i<1389; ++i) {

        img_proc.load_image(image_dir, i);
        img_proc.show_image(1);

    }
    return 0;

    int k = 0;
    double asd = 0;
    k++;
    cout<<k<<endl;
}