#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <iostream>


struct SURFDetector
{
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

    void detect_keypoints(cv::Mat & imgl, cv::Mat & imgr) {
        surf->detectAndCompute(imgl, cv::noArray(), l_keypt, l_des);
        surf->detectAndCompute(imgr, cv::noArray(), l_keypt, l_des);
    }


    void show_matches(cv::Mat & src, std::vector<cv::KeyPoint> & keypt){
        cv::Mat img_keypoints;
        cv::drawKeypoints(src, keypt, img_keypoints);
        cv::imshow("SURF Keypoints", img_keypoints);
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