#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
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
};