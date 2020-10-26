#include <iostream>
#include "calculate_transformations.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

class DrProcessing{
public:
    DrProcessing();
    ~DrProcessing();


    CalTransform c_trans;

    std::vector<std::vector<float>> nav_data;
    std::vector<std::vector<float>> rel_nav_data;


    void get_relative_pose(std::vector<float> & state_1, 
                           std::vector<float> & state_2, 
                           std::vector<float> * rel_pose1_2);
    void load_dr(std::string & filedir, int & data_num);

    void test(std::vector<float> a);

};