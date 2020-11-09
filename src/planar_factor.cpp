#include "planar_factor.h"

PlanarFactor::PlanarFactor(Key key1, Key key2, 
                           const Vector4& measured, const SharedNoiseModel& model):
                           NoiseModelFactor2(model, key1, key2),
                           measured_(measured){}

Vector PlanarFactor::evaluateError(const StatePlane& sp1, const StatePlane & sp2,
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

    Vector3 t1(sp1.x, sp1.y, sp1.z);
    Vector3 t2(sp2.x, sp2.y, sp2.z);

    Vector3 t12 = R1.transpose()*(t2-t1);
    Vector3 t21 = R2.transpose()*(t1-t2);


    Vector3 n1(sp1.nx, sp1.ny, sp1.nz);
    Vector3 n2(sp2.nx, sp2.ny, sp2.nz);

    double d1 = sp1.d;
    double d2 = sp2.d;

    //Estimated normals and distance
    Vector3 n21 = R21*n1;
    double d21 = n1.transpose()*R1.transpose()*(t2-t1)+d1;

    if(H1){
        Matrix H1_ = Matrix::Zero(4,10);
        H1_.block(0,3,3,1) = R2.transpose()*R1x*n1;
        H1_.block(0,4,3,1) = R2.transpose()*R1y*n1;
        H1_.block(0,5,3,1) = R2.transpose()*R1z*n1;
        H1_.block(0,6,3,3) = R2.transpose()*R1;
        H1_.block(3,0,1,3) = -n1.transpose()*R1.transpose();
        H1_.block(3,3,1,1) = n1.transpose()*R1x.transpose()*(t2-t1);
        H1_.block(3,4,1,1) = n1.transpose()*R1y.transpose()*(t2-t1);
        H1_.block(3,5,1,1) = n1.transpose()*R1z.transpose()*(t2-t1);
        H1_.block(3,6,1,3) = (t2-t1).transpose()*R1;
        H1_(3,9) = 1.0;
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(4,10);
        H2_.block(0,3,3,1) = R2x.transpose()*R1*n1;
        H2_.block(0,4,3,1) = R2y.transpose()*R1*n1;
        H2_.block(0,5,3,1) = R2z.transpose()*R1*n1;
        H2_.block(3,0,1,3) = n1.transpose()*R1.transpose();
        *H2 = H2_;
    }

    Vector4 result;
    result(0) = measured_(0) - n21(0);
    result(1) = measured_(1) - n21(1);
    result(2) = measured_(2) - n21(2);
    result(3) = measured_(3) - d21;

    return result;
}

Matrix PlanarFactor::rollMatrix(float roll, bool jacobian) const{

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

Matrix PlanarFactor::pitchMatrix(float pitch, bool jacobian) const{
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

Matrix PlanarFactor::yawMatrix(float yaw, bool jacobian) const{
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

Matrix PlanarFactor::rotationMatrix(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result = yawMatrix(yaw)*pitchMatrix(pitch)*rollMatrix(roll);
    return result;
}

Matrix PlanarFactor::rollJacobiMatrix(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result = yawMatrix(yaw)*pitchMatrix(pitch)*rollMatrix(roll, true);
    return result;
}
Matrix PlanarFactor::pitchJacobiMatrix(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result = yawMatrix(yaw)*pitchMatrix(pitch, true)*rollMatrix(roll);
    return result;
}
Matrix PlanarFactor::yawJacobiMatrix(float roll, float pitch, float yaw) const{
    Matrix result = zeros(3,3);
    result = yawMatrix(yaw, true)*pitchMatrix(pitch)*rollMatrix(roll);
    return result;
}
