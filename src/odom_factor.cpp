#include "odom_factor.h"

OdomFactor::OdomFactor(Key key1, Key key2, 
                           const Vector6& measured, const SharedNoiseModel& model):
                           NoiseModelFactor2(model, key1, key2),
                           measured_(measured){}

Vector OdomFactor::evaluateError(const StatePlane& sp1, const StatePlane & sp2,
                                   boost::optional<Matrix&> H1,
                                   boost::optional<Matrix&> H2) const{

    Matrix R1 = rotationMatrix(sp1.roll, sp1.pitch, sp1.yaw);
    Matrix R2 = rotationMatrix(sp2.roll, sp2.pitch, sp2.yaw);
    Matrix R12 = R1.transpose()*R2;
    Matrix R21 = R2.transpose()*R1;

    Matrix R1x = rollJacobiMatrix(sp1.roll, sp1.pitch, sp1.yaw);
    Matrix R1y = pitchJacobiMatrix(sp1.roll, sp1.pitch, sp1.yaw);
    Matrix R1z = yawJacobiMatrix(sp1.roll, sp1.pitch, sp1.yaw);

    Matrix R2x = rollJacobiMatrix(sp2.roll, sp2.pitch, sp2.yaw);
    Matrix R2y = pitchJacobiMatrix(sp2.roll, sp2.pitch, sp2.yaw);
    Matrix R2z = yawJacobiMatrix(sp2.roll, sp2.pitch, sp2.yaw);

    Matrix g2i = globalToInertial(sp1.roll, sp1.pitch, sp1.yaw);
    Matrix g2i_roll = rollJacobiGlobalToInertial(sp1.roll, sp1.pitch, sp1.yaw);
    Matrix g2i_pitch = pitchJacobiGlobalToInertial(sp1.roll, sp1.pitch, sp1.yaw);
    Matrix g2i_yaw = yawJacobiGlobalToInertial(sp1.roll, sp1.pitch, sp1.yaw);



    Vector3 t1(sp1.x, sp1.y, sp1.z);
    Vector3 t2(sp2.x, sp2.y, sp2.z);

    Vector3 t12 = R1.transpose()*(t2-t1);
    Vector3 t21 = R2.transpose()*(t1-t2);

    Vector3 a12 = Vector3(sp2.roll - sp1.roll,
                          sp2.pitch - sp2.pitch,
                          sp2.yaw - sp2.yaw);

    if(H1){
        Matrix H1_ = Matrix::Zero(6,10);
        H1_.block(0,0,3,3) = -R1.transpose();
        H1_.block(0,4,3,1) = R1x.transpose()*(t2-t1);
        H1_.block(0,5,3,1) = R1y.transpose()*(t2-t1);
        H1_.block(0,6,3,1) = R1z.transpose()*(t2-t1);
        
        H1_.block(3,3,3,1) = g2i_roll*a12 + g2i*Vector3(-1,0,0);
        H1_.block(3,4,3,1) = g2i_pitch*a12 + g2i*Vector3(0,-1,0);
        H1_.block(3,5,3,1) = g2i_yaw*a12 + g2i*Vector3(0,0,-1);

        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(6,10);
        H2_.block(0,0,3,1) = R1.transpose();
        H2_.block(3,3,3,3) = g2i;
        *H2 = H2_;
    }

    Vector6 result;
    result(0) = measured_(0) - t12(0);
    result(1) = measured_(1) - t12(1);
    result(2) = measured_(2) - t12(2);
    result(3) = 0;
    result(4) = 0;
    result(5) = 0;

    return result;
}

Matrix OdomFactor::rollMatrix(float roll, bool jacobian) const{

    Matrix result = zeros(3,3);
    if(!jacobian){
        result(0,0) = 1.0;
        result(1,1) = cos(roll);
        result(1,2) = -sin(roll);
        result(2,1) = sin(roll);
        result(2,2) = cos(roll);
    } else {
        result(1,1) = -sin(roll);
        result(1,2) = -cos(roll);
        result(2,1) = cos(roll);
        result(2,2) = -sin(roll);
    }
    return result;
}

Matrix OdomFactor::pitchMatrix(float pitch, bool jacobian) const{
    Matrix result = zeros(3,3);
    if(!jacobian){
        result(0,0) = cos(pitch);
        result(0,2) = sin(pitch);
        result(1,1) = 1.0;
        result(2,0) = -sin(pitch);
        result(2,2) = cos(pitch);
    } else {
        result(0,0) = -sin(pitch);
        result(0,2) = cos(pitch);
        result(2,0) = -cos(pitch);
        result(2,2) = -sin(pitch);
    }
    return result;
}

Matrix OdomFactor::yawMatrix(float yaw, bool jacobian) const{
    Matrix result = zeros(3,3);
    if(!jacobian){
        result(0,0) = cos(yaw);
        result(0,1) = -sin(yaw);
        result(1,0) = sin(yaw);
        result(1,1) = cos(yaw);
        result(2,2) = 1.0;
    } else {
        result(0,0) = -sin(yaw);
        result(0,1) = -cos(yaw);
        result(1,0) = cos(yaw);
        result(1,1) = -sin(yaw);
    }
    return result;
}

Matrix OdomFactor::rotationMatrix(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result = yawMatrix(yaw)*pitchMatrix(pitch)*rollMatrix(roll);
    return result;
}

Matrix OdomFactor::rollJacobiMatrix(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result = yawMatrix(yaw)*pitchMatrix(pitch)*rollMatrix(roll, true);
    return result;
}
Matrix OdomFactor::pitchJacobiMatrix(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result = yawMatrix(yaw)*pitchMatrix(pitch, true)*rollMatrix(roll);
    return result;
}
Matrix OdomFactor::yawJacobiMatrix(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result = yawMatrix(yaw, true)*pitchMatrix(pitch)*rollMatrix(roll);
    return result;
}

Matrix OdomFactor::globalToInertial(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result(0,0) = 1.0;
    result(0,2) = -sin(pitch);
    result(1,1) = cos(roll);
    result(1,2) = sin(roll)*cos(pitch);
    result(2,1) = -sin(roll);
    result(2,2) = cos(roll)*cos(pitch);
    return result;
}

Matrix OdomFactor::rollJacobiGlobalToInertial(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result(1,1) = -sin(roll);
    result(1,2) = cos(roll)*cos(pitch);
    result(2,1) = -cos(roll);
    result(2,2) = -sin(roll)*cos(pitch);
    return result;
}
Matrix OdomFactor::pitchJacobiGlobalToInertial(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result(0,2) = -cos(pitch);
    result(1,2) = -sin(roll)*sin(pitch);
    result(2,2) = -cos(roll)*sin(pitch);
    return result;
}
Matrix OdomFactor::yawJacobiGlobalToInertial(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3); 
    return result;  
}
