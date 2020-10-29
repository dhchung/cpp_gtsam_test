#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <algorithm>
#include "point_cloud.h"
#include <opencv2/core/eigen.hpp>
#include "parameters.h"


struct Matching_idx{
    int idx;
    float dist;
    Matching_idx(int index, float distance){
        idx = index;
        dist = distance;
    }
};

struct SURFDetector
{
    cv::Ptr<cv::Feature2D> surf;
    cv::Ptr<cv::Feature2D> extractor;
    std::vector<std::vector<int>> match_pairs;

    SURFDetector(float hessian = 200.0){
        int k = 0;
        surf = cv::xfeatures2d::SURF::create(hessian, 4, 3, false, true);
        extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    }

    void detectAndCompute_keypoints(cv::Mat & imgl, cv::Mat & imgr,
                                    std::vector<cv::KeyPoint> *l_keypt,
                                    std::vector<cv::KeyPoint> *r_keypt,
                                    cv::Mat *l_des,
                                    cv::Mat *r_des) {
        l_keypt->clear();
        r_keypt->clear();

        surf->detectAndCompute(imgl, cv::noArray(), *l_keypt, *l_des);
        surf->detectAndCompute(imgr, cv::noArray(), *r_keypt, *r_des);
    }
};

class ImageProcessing{
public:
    ImageProcessing();
    ~ImageProcessing();

    cv::Mat l_img;
    cv::Mat r_img;
    cv::Mat c_img;

    cv::Mat p_l_img;
    cv::Mat p_r_img;

    cv::Ptr<cv::CLAHE> clahe;
    
    SURFDetector surf;
    

    void load_image(std::string &img_dir, int &img_number);
    void apply_clahe(float clip_limit);
    int show_image(int wait);
    int show_image(cv::Mat &imgL, cv::Mat &imgR, int wait);

    void match_stereo(float & depth, PointCloud * pt_cld);
    
    void get_3d_points(std::vector<cv::KeyPoint> &l_keypt,
                       std::vector<cv::KeyPoint> &r_keypt,
                       cv::Mat & l_des,
                       std::vector<std::vector<int>> &matched_pairs,
                       PointCloud * pt_cld);

    void show_matches(std::vector<cv::KeyPoint> &l_keypt,
                      std::vector<cv::KeyPoint> &r_keypt,
                      std::vector<std::vector<int>> &matched_pairs);
};