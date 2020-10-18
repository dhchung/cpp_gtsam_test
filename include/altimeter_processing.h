#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

class AltimeterProcessing{
public:
    AltimeterProcessing();
    ~AltimeterProcessing();
    std::vector<double> depth;
    void load_altimeter(std::string &filedir, int & data_num);
};