#include <iostream>
#include "calculate_transformations.h"
#include <vector>

class DrProcessing{
public:
    DrProcessing();
    ~DrProcessing();

    void get_relative_pose(std::vector<float> & state_1, 
                           std::vector<float> & state_2, 
                           std::vector<float> * rel_pose1_2);

};