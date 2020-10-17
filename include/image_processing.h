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
    struct Matching_idx{
        int idx;
        double dist;
        Matching_idx(int index, double distance){
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

    SURFDetector(double hessian = 200.0){
        int k = 0;
        surf = cv::xfeatures2d::SURF::create(hessian, 4, 3, false, true);
        extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    }

    void detectAndCompute_keypoints(cv::Mat & imgl, cv::Mat & imgr) {
        surf->detectAndCompute(imgl, cv::noArray(), l_keypt, l_des);
        surf->detectAndCompute(imgr, cv::noArray(), r_keypt, r_des);
    }

    void show_matches(cv::Mat & src, std::vector<cv::KeyPoint> & keypt){
        cv::Mat img_keypoints;
        cv::drawKeypoints(src, keypt, img_keypoints);
        cv::imshow("SURF Keypoints", img_keypoints);
    }

    void match_stereo(double min_x_px_diff, double max_x_px_diff, double max_y_px_diff){
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
            double base_y = l_keypt[l_pt_idx].pt.y;
            double base_x = l_keypt[l_pt_idx].pt.x;
            double base_s = l_keypt[l_pt_idx].size;


            Eigen::VectorXd x_distance = r_feat_loc.row(1).array() - base_x;
            Eigen::VectorXd y_distance = r_feat_loc.row(2).array() - base_y;

            std::vector<int> logic_x_y_s;
            

            for(int i=0; i<r_feat_num; ++i){
                if(x_distance(i)>0)
                    continue;
                if(-x_distance(i) < min_x_px_diff || -x_distance(i) > max_x_px_diff)
                    continue;
                if(abs(y_distance(i))>max_y_px_diff)
                    continue;
                if(abs(r_keypt[i].size-base_s)>10)
                    continue;
                double distance = cv::norm(l_des.row(l_pt_idx), r_des.row(i));
                if(distance>0.5)
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
                        }
                    }
                    if(!is_min)
                        continue;
                }

                if(!l2r_candid[l_pt_idx].empty()){
                    bool is_min = false;
                    for(int j=0; j<l2r_candid[l_pt_idx].size(); ++j){
                        if(l2r_candid[l_pt_idx][j].dist>distance) {
                            is_min = true;

                        }
                    }
                }

                l2r_candid[l_pt_idx].push_back(Matching_idx(i, distance));
                logic_x_y_s.push_back(i);
                std::cout<<i<<std::endl;
            }


        }
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
    void apply_clahe(double clip_limit);
    int show_image(int wait);
    int show_image(cv::Mat &imgL, cv::Mat &imgR, int wait);
};