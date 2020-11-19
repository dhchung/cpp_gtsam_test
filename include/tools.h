#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>
#include "state_plane.h"
#include <vector>

class Tools{
public:
    gtsam::Vector turnVectorToGTSAMVector(std::vector<float>& vector){
        gtsam::Vector result;
        int size_of_vec = vector.size();
        result = gtsam::Vector(size_of_vec);
        for(int i = 0; i < size_of_vec; ++i) {
            result(i) = vector[i];
        }
        return result;
    };

    std::vector<float> turnGTSAMVectorToVector(gtsam::Vector& vec){
        std::vector<float> result;
        int size_of_vec = vec.rows();
        result.resize(size_of_vec);
        for(int i = 0; i< size_of_vec; ++i) {
            result[i] = vec(i);
        }
        return result;
    };

    void evaluateError(gtsam::Values & init, gtsam::Values & result){
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        double sum_roll = 0.0;
        double sum_pitch = 0.0;
        double sum_yaw = 0.0;
        double sum_nx = 0.0;
        double sum_ny = 0.0;
        double sum_nz = 0.0;
        double sum_d = 0.0;

        for(int i = 0; i < init.size(); ++i){
            gtsamexample::StatePlane init_state = init.at<gtsamexample::StatePlane>(i);
            gtsamexample::StatePlane result_state = result.at<gtsamexample::StatePlane>(i);


            sum_x += pow(init_state.x - result_state.x,2);
            sum_y += pow(init_state.y - result_state.y,2);
            sum_z += pow(init_state.z - result_state.z,2);
            sum_roll += pow(init_state.roll - result_state.roll,2);
            sum_pitch += pow(init_state.pitch - result_state.pitch,2);
            sum_yaw += pow(init_state.yaw - result_state.yaw,2);
            sum_nx += pow(init_state.nx - result_state.nx,2);
            sum_ny += pow(init_state.ny - result_state.ny,2);
            sum_nz += pow(init_state.nz - result_state.nz,2);
            sum_d += pow(init_state.d - result_state.d,2);

        }

        float data_size = init.size();

        float sum_all = sum_x+sum_y+sum_z+sum_roll+sum_pitch+sum_yaw+sum_nx+sum_ny+sum_nz+sum_d;
        float sum_position = sum_x+sum_y+sum_z;

        sum_all = sqrt(sum_all/data_size);
        sum_position = sqrt(sum_position/data_size);
        sum_x = sqrt(sum_x/data_size);
        sum_y = sqrt(sum_y/data_size);
        sum_z = sqrt(sum_z/data_size);
        sum_roll = sqrt(sum_roll/data_size);
        sum_pitch = sqrt(sum_pitch/data_size);
        sum_yaw = sqrt(sum_yaw/data_size);
        sum_nx = sqrt(sum_nx/data_size);
        sum_ny = sqrt(sum_ny/data_size);
        sum_nz = sqrt(sum_nz/data_size);
        sum_d = sqrt(sum_d/data_size);

        std::cout<<"Position Error_x : "<<sum_x<<std::endl;
        std::cout<<"Position Error_y : "<<sum_y<<std::endl;
        std::cout<<"Position Error_z : "<<sum_z<<std::endl;
        std::cout<<"Angle Error_roll : "<<sum_roll<<std::endl;
        std::cout<<"Angle Error_pitch : "<<sum_pitch<<std::endl;
        std::cout<<"Angle Error_yaw : "<<sum_yaw<<std::endl;
        std::cout<<"Normal Error_nx : "<<sum_nx<<std::endl;
        std::cout<<"Normal Error_ny : "<<sum_ny<<std::endl;
        std::cout<<"Normal Error_nz : "<<sum_nz<<std::endl;
        std::cout<<"Distance Error_d : "<<sum_d<<std::endl;

        std::cout<<"All Error : "<<sum_all<<std::endl;
        std::cout<<"Position Error : "<<sum_position<<std::endl;


    }
};