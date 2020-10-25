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

void ImageProcessing::match_stereo(float & depth, float & depth_err, PointCloud * pt_cld) {
    float short_depth = depth - depth_err;
    float long_depth = depth + depth_err;

    float max_pixel_diff = base_line*focal_length/short_depth;
    float min_pixel_diff = base_line*focal_length/long_depth;

    std::vector<cv::KeyPoint> l_keypt;
    std::vector<cv::KeyPoint> r_keypt;

    cv::Mat l_des;
    cv::Mat r_des;

    surf.detectAndCompute_keypoints(p_l_img, p_r_img, &l_keypt, &r_keypt, &l_des, &r_des);

    //Match Stereo

    int l_feat_num = l_keypt.size();
    int r_feat_num = r_keypt.size();

    Eigen::Matrix2Xd l_feat_loc(2,l_feat_num);
    Eigen::Matrix2Xd r_feat_loc(2,r_feat_num);

    for(int i=0; i<l_feat_num; ++i){
        l_feat_loc.col(i)<<l_keypt[i].pt.x, l_keypt[i].pt.y;
    }
    for(int i=0; i<r_feat_num; ++i){
        r_feat_loc.col(i)<<r_keypt[i].pt.x, r_keypt[i].pt.y;
    }

    std::vector<std::vector<Matching_idx>> l2r_candid;
    std::vector<std::vector<Matching_idx>> r2l_candid;

    l2r_candid.resize(l_feat_num);
    r2l_candid.resize(r_feat_num);

    for(int l_pt_idx = 0; l_pt_idx<l_feat_num; ++l_pt_idx){
        float base_y = l_keypt[l_pt_idx].pt.y;
        float base_x = l_keypt[l_pt_idx].pt.x;
        float base_s = l_keypt[l_pt_idx].size;


        Eigen::VectorXd x_distance = r_feat_loc.row(0).array() - base_x;
        Eigen::VectorXd y_distance = r_feat_loc.row(1).array() - base_y;

        for(int i=0; i<r_feat_num; ++i){
            if(x_distance(i)>0)
                continue;

            if(-x_distance(i) < min_pixel_diff)
                continue;

            if(-x_distance(i) > max_pixel_diff)
                continue;

            if(abs(y_distance(i))>5.0)
                continue;

            if(abs(r_keypt[i].size-base_s)>5)
                continue;

            float distance = cv::norm(l_des.row(l_pt_idx), r_des.row(i));
            if(distance>0.4)
                continue;

            if(!r2l_candid[i].empty()){
                bool is_min = false;
                for(int j=0; j<r2l_candid[i].size(); ++j){
                    if(r2l_candid[i][j].dist>distance) {
                        is_min = true;
                        for(int k=0; k<l2r_candid[r2l_candid[i][j].idx].size();++k){
                            if(l2r_candid[r2l_candid[i][j].idx][k].idx==i){
                                l2r_candid[r2l_candid[i][j].idx].erase(l2r_candid[r2l_candid[i][j].idx].begin()+k);
                            }
                        }
                        r2l_candid[i].erase(r2l_candid[i].begin()+j);
                    } else{
                        is_min = false;
                    }
                }
                if(!is_min)
                    continue;
            }

            if(!l2r_candid[l_pt_idx].empty()){
                bool is_min = false;
                for(int j=0; j<l2r_candid[l_pt_idx].size(); ++j){
                    float cur_dist = l2r_candid[l_pt_idx][j].dist;
                    if(cur_dist>distance) {
                        is_min = true;
                        for(int k=0; k<r2l_candid[l2r_candid[l_pt_idx][j].idx].size();++k) {
                            if(r2l_candid[l2r_candid[l_pt_idx][j].idx][k].idx==l_pt_idx){
                                r2l_candid[l2r_candid[l_pt_idx][j].idx].erase(r2l_candid[l2r_candid[l_pt_idx][j].idx].begin()+k);
                            }
                        }
                        l2r_candid[l_pt_idx].erase(l2r_candid[l_pt_idx].begin()+j);
                        // if(cur_dist/distance<0.8) {
                        //     is_min = false;
                        // }
                        
                    } else{
                        is_min = false;
                    }
                }
                if(!is_min){
                    continue;
                }
            }

            l2r_candid[l_pt_idx].push_back(Matching_idx(i, distance));
            r2l_candid[i].push_back(Matching_idx(l_pt_idx, distance));
        }
    }

    std::vector<std::vector<int>> matched_pairs;
    for(int i=0; i<l2r_candid.size(); ++i){
        if(!l2r_candid[i].empty()){
            matched_pairs.push_back(std::vector<int>{i, l2r_candid[i][0].idx});
        }
    }

    get_3d_points(l_keypt, r_keypt, l_des, matched_pairs, pt_cld);
}

void ImageProcessing::get_3d_points(std::vector<cv::KeyPoint> &l_keypt,
                                    std::vector<cv::KeyPoint> &r_keypt,
                                    cv::Mat &l_des,
                                    std::vector<std::vector<int>> &matched_pairs,
                                    PointCloud * pt_cld){

    pt_cld->point_cloud.resize(3, matched_pairs.size());
    pt_cld->point_color.resize(3, matched_pairs.size());
    pt_cld->point_des.resize(64, matched_pairs.size());
    pt_cld->point_size.resize(matched_pairs.size());

    for(int i=0; i<matched_pairs.size(); i++) {
        float x_l = l_keypt[matched_pairs[i][0]].pt.x;
        float x_r = r_keypt[matched_pairs[i][1]].pt.x;
        float y_l = l_keypt[matched_pairs[i][0]].pt.y;
        
        float disparity = x_l-x_r;

        pt_cld->point_cloud.col(i) << base_line/disparity*focal_length,
                                     base_line/disparity*(x_l - img_center_x),
                                     base_line/disparity*(y_l - img_center_y);

        cv::Vec3b color = l_img.at<cv::Vec3b>(cv::Point(int(x_l), int(y_l)));
        pt_cld->point_color.col(i) << color[2], color[1], color[0];
        pt_cld->point_size(i) = l_keypt[matched_pairs[i][0]].size;
        Eigen::MatrixXf des;
        cv::cv2eigen(l_des.row(matched_pairs[i][0]), des);
        pt_cld->point_des.col(i) = des.transpose();
    }

    show_matches(l_keypt, r_keypt, matched_pairs);

}

void ImageProcessing::show_matches(std::vector<cv::KeyPoint> &l_keypt,
                                   std::vector<cv::KeyPoint> &r_keypt,
                                   std::vector<std::vector<int>> &matched_pairs){
    cv::Mat l_img_copy = l_img.clone();
    cv::Mat r_img_copy = r_img.clone();


    for(auto & pt:l_keypt) {
        circle(l_img_copy,
                pt.pt,
                2,
                cv::Scalar(255,0,0),
                cv::FILLED,
                cv::LINE_8);
    }
    for(auto & pt:r_keypt) {
        circle(r_img_copy,
                pt.pt,
                2,
                cv::Scalar(0,0,255),
                cv::FILLED,
                cv::LINE_8);
    }

    cv::Mat combined_img;
    cv::hconcat(l_img_copy, r_img_copy, combined_img);

    cv::Point2f Img_width_pt(l_img.cols, 0);

    for(auto & pairs:matched_pairs){
        line(combined_img,
                l_keypt[pairs[0]].pt,
                r_keypt[pairs[1]].pt+Img_width_pt,
                cv::Scalar(255,255,255));
    }
    cv::imshow("Test", combined_img);
    cv::waitKey(1);
}