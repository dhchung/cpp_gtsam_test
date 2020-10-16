#include "image_processing.h"
ImageProcessing::ImageProcessing(){
    clahe = cv::createCLAHE();
    
}
ImageProcessing::~ImageProcessing(){

}

void ImageProcessing::load_image(std::string &img_dir, int &img_number){
    l_img = cv::imread(img_dir+"/Left/img"+std::to_string(img_number)+".jpg");
    r_img = cv::imread(img_dir+"/Right/img"+std::to_string(img_number)+".jpg");
}

int ImageProcessing::show_image(int wait){
    if(l_img.empty()){
        return 0;
    }
    if(r_img.empty()){
        return 0;
    }

    cv::hconcat(l_img, r_img, c_img);


    if(!cv::getWindowProperty("Stereo images", cv::WND_PROP_VISIBLE)) {
        cv::namedWindow("Stereo images", cv::WINDOW_AUTOSIZE);
    }

    cv::imshow("Stereo images", c_img);
    cv::waitKey(wait);

}

int ImageProcessing::show_image(cv::Mat &imgL, cv::Mat &imgR, int wait){
    if(imgL.empty()){
        return 0;
    }
    if(imgR.empty()){
        return 0;
    }
    cv::hconcat(imgL, imgR, c_img);
    if(!cv::getWindowProperty("Stereo images", cv::WND_PROP_VISIBLE)) {
        cv::namedWindow("Stereo images", cv::WINDOW_AUTOSIZE);
    }

    cv::imshow("Stereo images", c_img);
    cv::waitKey(wait);

}


void ImageProcessing::apply_clahe(double clip_limit){
    clahe->setClipLimit(clip_limit);
    cv::Mat g_l_img;
    cv::Mat g_r_img;
    
    cv::cvtColor(l_img, g_l_img, cv::COLOR_RGB2GRAY);
    cv::cvtColor(r_img, g_r_img, cv::COLOR_RGB2GRAY);

    clahe->apply(g_l_img, p_l_img);
    clahe->apply(g_r_img, p_r_img);
    
    surf.detectAndCompute_keypoints(p_l_img, p_r_img);
    surf.show_matches(l_img, surf.l_keypt);
    surf.match_stereo(0,0,0);
}

