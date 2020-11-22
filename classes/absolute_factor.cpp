#include "absolute_factor.h"

AbsoluteFactor::AbsoluteFactor(Key key1, 
                               const Vector3& measured, const SharedNoiseModel& model):
                               NoiseModelFactor1(model, key1),
                               measured_(measured){}

Vector AbsoluteFactor::evaluateError(const StatePlane& sp1,
                                     boost::optional<Matrix&> H) const{


    if(H){
        Matrix H_ = Matrix::Zero(3,10);
        H_(0,2)=1.0;
        H_(1,3)=1.0;
        H_(2,4)=1.0;
        *H = H_;
    }

    Vector3 result;
    result(0) = sp1.z - measured_(0);
    result(1) = sp1.roll - measured_(1);
    result(2) = sp1.pitch - measured_(2);
    return result;
}
