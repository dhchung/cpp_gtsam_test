#include "ransac_plane.h"

RANSACPlane::RANSACPlane(){

}
RANSACPlane::~RANSACPlane(){

}

void RANSACPlane::perform_ransac_plane(PointCloud & input,
                                       PointCloud * output){
    int input_data_size = input.point_cloud.cols();
    int max_iteration = 500;
    float inlier_p = 0.9;
    int i = 0;

    std::vector<int> max_inlier_idx;

    while(i < max_iteration){
        std::vector<int> sample_idx;
        select_index(input_data_size, &sample_idx);
        Eigen::Matrix3f sample_point;
        for(int k=0; k<3; ++k){
            sample_point.col(k)=input.point_cloud.col(sample_idx[k]);
        }
        // Eigen::JacobiSVD<Eigen::MatrixXf> svd(sample_point);
        // double cond = svd.singularValues()(0)/svd.singularValues()(svd.singularValues().size()-1);
        if(sample_point.determinant()==0){
            continue;
        }
        Eigen::Vector3f model_param = -sample_point.transpose().inverse()*Eigen::Vector3f::Ones();
        Eigen::VectorXf distance;

        Eigen::Matrix4Xf temp(4, input.point_cloud.cols());
        temp.block(0,0,3,input.point_cloud.cols()) = input.point_cloud;
        temp.row(3) = Eigen::VectorXf::Ones(input.point_cloud.cols());
        Eigen::Vector4f temp2;
        temp2<<model_param, 1;
        distance = temp.transpose()*temp2/(model_param.norm());

        distance = distance.array().abs();

        std::vector<int> indices;
        float distance_threshold = 50;
        get_indices(distance, &indices, distance_threshold);    


        if(indices.size()>max_inlier_idx.size()){
            max_inlier_idx.clear();
            max_inlier_idx = indices;
        }
        if(max_inlier_idx.size()>0) {
            float e = 1-(float(max_inlier_idx.size())/float(input_data_size));
            max_iteration = int(log10(1- inlier_p)/log10(1-pow((1-e), 3)));
        }
        ++i;
    }
    
    output->point_cloud.resize(3, max_inlier_idx.size());
    output->point_color.resize(3, max_inlier_idx.size());
    output->point_des.resize(64, max_inlier_idx.size());
    output->point_size.resize(max_inlier_idx.size());

    for(int i=0; i<max_inlier_idx.size(); ++i) {
        output->point_cloud.col(i) = input.point_cloud.col(max_inlier_idx[i]);
        output->point_color.col(i) = input.point_color.col(max_inlier_idx[i]);
        output->point_des.col(i) = input.point_des.col(max_inlier_idx[i]);
        output->point_size(i) = input.point_size(i);
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


void RANSACPlane::get_indices(Eigen::VectorXf & distance, 
                              std::vector<int> * idx, 
                              float & distnace_threshold){
    idx->clear();
    for(int i=0; i<distance.size(); ++i){
        if(distance(i)<distnace_threshold){
            idx->push_back(i);
        }
    }
}