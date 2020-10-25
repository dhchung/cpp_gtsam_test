#include "calculate_transformations.h"

CalTransform::CalTransform(){
}

CalTransform::~CalTransform(){

}

void CalTransform::xyzrpy2t(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Matrix4f * T){

    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    T->setZero(4,4);
    T->block(0,0,3,3) = R;
    T->operator()(3,3) = 1.0f;

}


void CalTransform::t2xyzrpy(Eigen::Matrix4f T, std::vector<float> * xyzrpy){

    Eigen::Matrix3f R = T.block(0,0,3,3);
    std::vector<float> rpy;
    r2rpy(R, &rpy);
    Eigen::Vector3f xyz = T.block(0,3,3,1);

    xyzrpy->clear();
    xyzrpy->push_back(xyz(0));
    xyzrpy->push_back(xyz(1));
    xyzrpy->push_back(xyz(2));
    xyzrpy->insert(xyzrpy->end(), rpy.begin(), rpy.end());

}

void CalTransform::rpy2r(float roll, float pitch, float yaw, Eigen::Matrix3f * R){

    Eigen::Matrix3f R_roll;
    R_roll << 1.0f, 0.0f, 0.0f,
              0.0f, cos(roll), -sin(roll),
              0.0f, sin(roll), cos(roll);

    Eigen::Matrix3f R_pitch;
    R_pitch << cos(pitch), 0.0f, sin(pitch),
               0.0f, 1.0f, 0.0f,
               -sin(pitch), 0.0f, cos(pitch);
   
    Eigen::Matrix3f R_yaw;
    R_yaw << cos(yaw), -sin(yaw), 0.0f,
             sin(yaw), cos(yaw), 0.0f,
             0.0f, 0.0f, 1.0f;

    R->operator=(R_yaw*R_pitch*R_roll);

}

void CalTransform::r2rpy(Eigen::Matrix3f R, std::vector<float> * rpy){

    float roll;
    float pitch;
    float yaw;


    if(R(3,1) != 1.0f || R(3,1) != -1.0f) {
        pitch = -asin(R(2,0));
        roll = atan2(R(2,1)/cos(pitch), R(2,2)/cos(pitch));
        yaw = atan2(R(1,0)/cos(pitch), R(0,0)/cos(pitch));
    } else {
        yaw = 0.0f;
        if(R(2,0)==-1){
            pitch = M_PI/2.0f;
            roll = yaw + atan2(R(0,1), R(0,2));
        } else {
            pitch = -M_PI/2.0f;
            roll = -yaw + atan2(-R(0,1), -R(0,2));
        }
    }

    rpy->push_back(roll);
    rpy->push_back(pitch);
    rpy->push_back(yaw);
}

void CalTransform::inverse_t(Eigen::Matrix4f T1, Eigen::Matrix4f * T2){
    T2->setZero(4,4);
    Eigen::Matrix3f R = T1.block(0,0,3,3);
    Eigen::Vector3f trans = T1.block(0,3,3,1);

    T2->block(0,0,3,3) = R.transpose();
    T2->block(0,3,3,1) = -R.transpose()*trans;
    T2->operator()(3,3) = 1.0f;
}

Eigen::Matrix4f CalTransform::inverse_t(Eigen::Matrix4f T){
    Eigen::Matrix3f R = T.block(0,0,3,3);
    Eigen::Vector3f trans = T.block(0,3,3,1);

    Eigen::Matrix4f result = Eigen::Matrix4f::Zero(4,4);

    result.block(0,0,3,3) = R.transpose();
    result.block(0,3,3,1) = -R.transpose()*trans;
    result(3,3) = 1.0f;
    return result;
}