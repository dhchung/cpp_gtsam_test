#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <algorithm>

struct SURFDetector
{
    struct SurfPoint3D{
        // float loc_l;
        // float loc_r;
        // float disparity;
        float scale;
        // cv::Mat descriptor;
        float loc_3d_x;
        float loc_3d_y;
        float loc_3d_z;
        int r;
        int g;
        int b;
    };

    struct Matching_idx{
        int idx;
        float dist;
        Matching_idx(int index, float distance){
            idx = index;
            dist = distance;
        }
    };
    cv::Ptr<cv::Feature2D> surf;
    cv::Ptr<cv::Feature2D> extractor;

    std::vector<cv::KeyPoint> l_keypt;
    std::vector<cv::KeyPoint> r_keypt;

    cv::Mat l_des;
    cv::Mat r_des;

    std::vector<std::vector<Matching_idx>> l2r_candid;
    std::vector<std::vector<Matching_idx>> r2l_candid;

    std::vector<std::vector<int>> match_pairs;

    std::vector<SurfPoint3D> point_3d;

    SURFDetector(float hessian = 200.0){
        int k = 0;
        surf = cv::xfeatures2d::SURF::create(hessian, 4, 3, false, true);
        extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    }

    void detectAndCompute_keypoints(cv::Mat & imgl, cv::Mat & imgr) {
        l_keypt.clear();
        r_keypt.clear();

        surf->detectAndCompute(imgl, cv::noArray(), l_keypt, l_des);
        surf->detectAndCompute(imgr, cv::noArray(), r_keypt, r_des);
    }


    void match_stereo(float min_x_px_diff, float max_x_px_diff, float max_y_px_diff){
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

        std::vector<std::vector<Matching_idx>> l2r_candid_mem;
        std::vector<std::vector<Matching_idx>> r2l_candid_mem;

        l2r_candid_mem.resize(l_feat_num);
        r2l_candid_mem.resize(r_feat_num);

        for(int l_pt_idx = 0; l_pt_idx<l_feat_num; ++l_pt_idx){
            float base_y = l_keypt[l_pt_idx].pt.y;
            float base_x = l_keypt[l_pt_idx].pt.x;
            float base_s = l_keypt[l_pt_idx].size;


            Eigen::VectorXd x_distance = r_feat_loc.row(0).array() - base_x;
            Eigen::VectorXd y_distance = r_feat_loc.row(1).array() - base_y;

            for(int i=0; i<r_feat_num; ++i){
                if(x_distance(i)>0)
                    continue;

                if(-x_distance(i) < min_x_px_diff)
                    continue;

                if(-x_distance(i) > max_x_px_diff)
                    continue;

                if(abs(y_distance(i))>max_y_px_diff)
                    continue;

                if(abs(r_keypt[i].size-base_s)>10)
                    continue;
                    
                float distance = cv::norm(l_des.row(l_pt_idx), r_des.row(i));
                if(distance>0.5)
                    continue;
                if(!r2l_candid_mem[i].empty()){
                    bool is_min = false;
                    for(int j=0; j<r2l_candid_mem[i].size(); ++j){
                        if(r2l_candid_mem[i][j].dist>distance) {
                            is_min = true;
                            for(int k=0; k<l2r_candid_mem[r2l_candid_mem[i][j].idx].size();++k){
                                if(l2r_candid_mem[r2l_candid_mem[i][j].idx][k].idx==i){
                                    l2r_candid_mem[r2l_candid_mem[i][j].idx].erase(l2r_candid_mem[r2l_candid_mem[i][j].idx].begin()+k);
                                }
                            }
                            r2l_candid_mem[i].erase(r2l_candid_mem[i].begin()+j);
                        } else{
                            is_min = false;
                        }
                    }
                    if(!is_min)
                        continue;
                }

                if(!l2r_candid_mem[l_pt_idx].empty()){
                    bool is_min = false;
                    for(int j=0; j<l2r_candid_mem[l_pt_idx].size(); ++j){
                        if(l2r_candid_mem[l_pt_idx][j].dist>distance) {
                            is_min = true;
                            for(int k=0; k<r2l_candid_mem[l2r_candid_mem[l_pt_idx][j].idx].size();++k) {
                                if(r2l_candid_mem[l2r_candid_mem[l_pt_idx][j].idx][k].idx==l_pt_idx){
                                    r2l_candid_mem[l2r_candid_mem[l_pt_idx][j].idx].erase(r2l_candid_mem[l2r_candid_mem[l_pt_idx][j].idx].begin()+k);
                                }
                            }
                            l2r_candid_mem[l_pt_idx].erase(l2r_candid_mem[l_pt_idx].begin()+j);
                            
                        } else{
                            is_min = false;
                        }
                    }
                    if(!is_min){
                        continue;
                    }
                }

                l2r_candid_mem[l_pt_idx].push_back(Matching_idx(i, distance));
            }
        }
        l2r_candid = l2r_candid_mem;
        r2l_candid = r2l_candid_mem;

        match_pairs.clear();

        for(int i=0; i<l2r_candid.size(); ++i){
            if(!l2r_candid[i].empty()){
                match_pairs.push_back(std::vector<int>{i, l2r_candid[i][0].idx});
            }
        }
    }

    void show_matches(cv::Mat &ImgL, cv::Mat &ImgR){
        cv::Mat l_img_copy = ImgL.clone();
        cv::Mat r_img_copy = ImgR.clone();


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

        cv::Point2f Img_width_pt(ImgL.cols, 0);

        for(auto & pairs:match_pairs){
            line(combined_img,
                 l_keypt[pairs[0]].pt,
                 r_keypt[pairs[1]].pt+Img_width_pt,
                 cv::Scalar(255,255,255));
        }
        cv::imshow("Test", combined_img);
    }
};

class ImageProcessing{
public:
    ImageProcessing();
    ~ImageProcessing();
    float base_line;
    float img_center_x;
    float img_center_y;
    float focal_length;


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

    void match_stereo(float & depth, float & depth_err);
    void get_3d_points();
    // void match_stereo(float distance, )
};