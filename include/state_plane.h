#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <iostream>




namespace gtsamexample{
    struct StatePlane{
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        double nx;
        double ny;
        double nz;
        double d;

    StatePlane(double xi, double yi, double zi, 
               double rolli, double pitchi, double yawi, 
               double nxi, double nyi, double nz1, double di):
               x(xi), y(yi), z(zi),
               roll(rolli), pitch(pitchi), yaw(yawi),
               nx(nxi), ny(nyi), nz(nz1), d(di){}
    };
}



namespace gtsam{
    template<>
    struct traits<gtsamexample::StatePlane>{

        static void Print(const gtsamexample::StatePlane & m, const std::string & str = ""){
            std::cout<<str<<"("<<m.x<<", "<<m.y<<", "<<m.z<<std::endl<<", "
                    <<m.roll<<", "<<m.pitch<<", "<<m.yaw<<std::endl<<", "
                    <<m.nx<<", "<<m.ny<<", "<<m.nz<<", "<<m.d<<")"<<std::endl;
        }

        static bool Equals(const gtsamexample::StatePlane &m1, const gtsamexample::StatePlane &m2, double tol = 1e-8){
            if(fabs(m1.x-m2.x)<tol &&
            fabs(m1.y-m2.y)<tol &&
            fabs(m1.z-m2.z)<tol &&
            fabs(m1.roll-m2.roll)<tol &&
            fabs(m1.pitch-m2.pitch)<tol &&
            fabs(m1.yaw-m2.yaw)<tol &&
            fabs(m1.nx-m2.nx)<tol &&
            fabs(m1.ny-m2.ny)<tol &&
            fabs(m1.nz-m2.nz)<tol &&
            fabs(m1.d-m2.d)<tol){
                return true;
            }else{
                    return false;
            }
        }

        enum{dimension = 10};
        static int GetDimension(const gtsamexample::StatePlane&) {return dimension;}

        typedef gtsamexample::StatePlane ManifoldType;
        typedef Eigen::Matrix<double, dimension, 1> TangentVector;

        static TangentVector Local(const gtsamexample::StatePlane& origin,
                                   const gtsamexample::StatePlane& other){

            Vector10 result;
            result(0) = other.x-origin.x;
            result(1) = other.y-origin.y;
            result(2) = other.z-origin.z;
            result(3) = other.roll-origin.roll;
            result(4) = other.pitch-origin.pitch;
            result(5) = other.yaw-origin.yaw;
            result(6) = other.nx-origin.nx;
            result(7) = other.ny-origin.ny;
            result(8) = other.nz-origin.nz;
            result(9) = other.d-origin.d;

            return result;
        }

        static gtsamexample::StatePlane Retract(const gtsamexample::StatePlane& origin,
                                                    const TangentVector& v){
            return gtsamexample::StatePlane(origin.x+v(0),
                                            origin.y+v(1),
                                            origin.z+v(2),
                                            origin.roll+v(3),
                                            origin.pitch+v(4),
                                            origin.yaw+v(5),
                                            origin.nx+v(6),
                                            origin.ny+v(7),
                                            origin.nz+v(8),
                                            origin.d+v(9));                                            
        }

    };
}

