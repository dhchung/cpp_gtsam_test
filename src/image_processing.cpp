#include "image_processing.h"
ImageProcessing::ImageProcessing(){
    clahe = cv::createCLAHE();
    base_line = 120.0;
    img_center_x = 236.0;
    img_center_y = 172.0;
    focal_length = 526.0;

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
    return 0;

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
    return 0;

}


void ImageProcessing::apply_clahe(float clip_limit){
    clahe->setClipLimit(clip_limit);
    cv::Mat g_l_img;
    cv::Mat g_r_img;
    
    cv::cvtColor(l_img, g_l_img, cv::COLOR_RGB2GRAY);
    cv::cvtColor(r_img, g_r_img, cv::COLOR_RGB2GRAY);

    clahe->apply(g_l_img, p_l_img);
    clahe->apply(g_r_img, p_r_img);
    
}

void ImageProcessing::match_stereo(float & depth, float & depth_err) {
    float short_depth = depth - depth_err;
    float long_depth = depth + depth_err;

    float max_pixel_diff = base_line*focal_length/short_depth;
    float min_pixel_diff = base_line*focal_length/long_depth;

    surf.detectAndCompute_keypoints(p_l_img, p_r_img);
    surf.match_stereo(min_pixel_diff, max_pixel_diff, 10);

    

    surf.show_matches(l_img, r_img);
    // cv::waitKey(0);
}

void ImageProcessing::get_3d_points(){
    surf.point_3d.clear();
    surf.point_3d.resize(surf.match_pairs.size());
    for(int i=0; i<surf.match_pairs.size(); i++) {
        float x_l = surf.l_keypt[surf.match_pairs[i][0]].pt.x;
        float x_r = surf.r_keypt[surf.match_pairs[i][1]].pt.x;
        float y_l = surf.l_keypt[surf.match_pairs[i][0]].pt.y;
        
        float disparity = x_l-x_r;


        surf.point_3d[i].loc_3d_x = base_line/disparity*focal_length;
        surf.point_3d[i].loc_3d_y = base_line/disparity*(x_l - img_center_x);
        surf.point_3d[i].loc_3d_z = base_line/disparity*(y_l - img_center_y);


        surf.point_3d[i].scale = surf.l_keypt[surf.match_pairs[i][0]].size;
        cv::Vec3b color = l_img.at<cv::Vec3b>(cv::Point(int(x_l), int(y_l)));
        surf.point_3d[i].r = color[0];
        surf.point_3d[i].g = color[1];
        surf.point_3d[i].b = color[2];
    }
}

