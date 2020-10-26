#include "dr_processing.h"

DrProcessing::DrProcessing(){

}
DrProcessing::~DrProcessing(){

}

void DrProcessing::get_relative_pose(std::vector<float> & state_1, 
                                     std::vector<float> & state_2, 
                                     std::vector<float> * rel_pose1_2){
    rel_pose1_2->clear();

    Eigen::Matrix4f T1;
    Eigen::Matrix4f T2;
    Eigen::Matrix4f T12;

    c_trans.xyzrpy2t(state_1, &T1);
    c_trans.xyzrpy2t(state_2, &T2);
    T12 = c_trans.inverse_t(T1)*T2;
    c_trans.t2xyzrpy(T12, rel_pose1_2);

}

void DrProcessing::test(std::vector<float> a){
    for(int i = 0; i<a.size(); ++i){
        std::cout<<a[i]<<"\t";
    }
    std::cout<<std::endl;
}

void DrProcessing::load_dr(std::string & filedir, int & data_num){
    std::ifstream infile(filedir+"/navigation_log_file.txt");
    nav_data.clear();
    nav_data.resize(data_num);

    rel_nav_data.clear();
    rel_nav_data.resize(data_num-1);
    std::vector<std::string> record;

    while(infile){
        std::string s;
        if(!getline(infile, s))
            break;
        std::istringstream ss(s);
        while(ss){
            std::string s;
            if(!getline(ss, s, '\t'))
                break;
            if(!s.empty()){
                record.push_back(s);
            }
        }
    }

    for(int i=0; i<record.size(); ++i) {
        int data_num = i/7;
        int data_idx = i%7;
        if(data_idx==6){
            continue;
        }
        nav_data[data_num].push_back(std::stof(record[i]));
    }

    for(int i=0; i<rel_nav_data.size(); ++i){
        get_relative_pose(nav_data[i], nav_data[i+1], &rel_nav_data[i]);
    }

}
