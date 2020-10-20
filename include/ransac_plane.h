#include <vector>
#include <cstdlib>
#include <iostream>

class RANSACPlane{
public:
    RANSACPlane();
    ~RANSACPlane();

    void select_index(int & data_num, std::vector<int> *indices);

    void perform_ransac_plane(std::vector<std::vector<float>> &input,
                              std::vector<std::vector<float>> *output);
};