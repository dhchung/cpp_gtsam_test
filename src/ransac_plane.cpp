#include "ransac_plane.h"

RANSACPlane::RANSACPlane(){

}
RANSACPlane::~RANSACPlane(){

}

void RANSACPlane::perform_ransac_plane(std::vector<std::vector<float>> & input,
                                       std::vector<std::vector<float>> * output){
    int input_data_size = input.size();
    int max_iteration = 500;


    //Select three random index
    int idx = 0;
    std::vector<int> selected_idx;

    select_index(input_data_size, &selected_idx);


    for(int i=0;i<selected_idx.size(); ++i) {
        std::cout<<selected_idx[i]<<std::endl;
    }

}

void RANSACPlane::select_index(int & data_num, std::vector<int> *indices) {

    indices->clear();
    int idx = 0;
    while(idx < 3){
        bool selected = true;
        int k = rand()%data_num;
        for(int i=0;i<indices->size();++i) {
            if(k==indices->at(i)) {
                selected = false;;
            }
        }
        if(selected){
            indices->push_back(k);
            ++idx;
        } else{
            continue;
        }
        
    }

}