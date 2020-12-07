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
    T->block(0,3,3,1) = trans;
    T->operator()(3,3) = 1.0f;

}

void CalTransform::xyzrpy2t(std::vector<float> state, Eigen::Matrix4f * T){

    float x = state[0];
    float y = state[1];
    float z = state[2];
    float roll = state[3];
    float pitch = state[4];
    float yaw = state[5];
    
    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    T->setZero(4,4);
    T->block(0,0,3,3) = R;
    T->block(0,3,3,1) = trans;
    T->operator()(3,3) = 1.0f;

}


Eigen::Matrix4f CalTransform::xyzrpy2t(float x, float y, float z, float roll, float pitch, float yaw){

    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    Eigen::Matrix4f T;

    T.setZero(4,4);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = trans;
    T(3,3) = 1.0f;

    return T;
}

Eigen::Matrix4f CalTransform::xyzrpy2t(std::vector<float> state){

    float x = state[0];
    float y = state[1];
    float z = state[2];
    float roll = state[3];
    float pitch = state[4];
    float yaw = state[5];
    
    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    Eigen::Matrix4f T;

    T.setZero(4,4);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = trans;
    T(3,3) = 1.0f;
    return T;
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

Eigen::Vector4f CalTransform::transform_plane(Eigen::Matrix4f & T1, Eigen::Vector4f & p1, Eigen::Matrix4f & T2){
    Eigen::Vector3f n1 = p1.segment(0,3);
    float d1 = p1(3);

    Eigen::Matrix3f R1 = T1.block(0,0,3,3);
    Eigen::Vector3f t1 = T1.block(0,3,3,1);

    Eigen::Matrix3f R2 = T1.block(0,0,3,3);
    Eigen::Vector3f t2 = T1.block(0,3,3,1);

    Eigen::Vector3f n2 = R2.transpose()*R1*n1;
    float d2 = d1 + (t2-t1).transpose()*R1*n1;

    Eigen::Vector4f result;
    result<<n2, d2;
    return result;
}

Eigen::VectorXf CalTransform::transform_plane2surfel(Eigen::Matrix4f & T, Eigen::Vector4f & plane){

    Eigen::Vector3f n1 = plane.segment(0,3);
    Eigen::Vector4f d1_pt = Eigen::Vector4f::Zero(4);
    d1_pt(0) = -plane(3)/plane(0);
    d1_pt(3) = 1.0f;


    Eigen::Matrix3f R1 = T.block(0,0,3,3);
    Eigen::Vector3f t1 = T.block(0,3,3,1);

    Eigen::Vector3f n2 = R1*n1;
    Eigen::Vector4f c2_temp = T*d1_pt;
    Eigen::Vector3f c2 = c2_temp.segment(0,3);

    Eigen::VectorXf result(6);
    result<<n2, c2;
    return result;
}



Eigen::Vector3f CalTransform::transform_point(Eigen::Matrix4f &T, Eigen::Vector3f &pt){
    Eigen::Vector4f pt_1;
    pt_1<<pt, 1.0;
    Eigen::Vector3f result;
    Eigen::Vector4f pt_2;

    pt_2 = T*pt_1;
    result = pt_2.segment(0,3);
    return result;
}

Eigen::Matrix3Xf CalTransform::transform_points(Eigen::Matrix4f &T, Eigen::Matrix3Xf &pts){
    Eigen::Matrix4Xf pt_1 = Eigen::Matrix4Xf::Ones(4,pts.cols());
    pt_1.block(0,0,3,pts.cols()) = pts;
    Eigen::Matrix4Xf pt_2 = T*pt_1;
    Eigen::Matrix3Xf result = pt_2.block(0,0,3,pts.cols());
    return result;
}


void CalTransform::odometry_calculation(float & prev_x, float & prev_y, float & prev_z, float & prev_roll, float & prev_pitch, float & prev_yaw,
                                        float & cur_dx, float & cur_dy, float & cur_dz, float & cur_droll, float & cur_dpitch, float & cur_dyaw,
                                        std::vector<float>* cur_state){
    
}