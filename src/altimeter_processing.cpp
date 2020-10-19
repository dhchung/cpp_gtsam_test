#include "altimeter_processing.h"

AltimeterProcessing::AltimeterProcessing(){

}
AltimeterProcessing::~AltimeterProcessing(){

}


void AltimeterProcessing::load_altimeter(std::string & filedir, int & data_num){
    std::ifstream infile(filedir+"/altimeter_log_file.txt");
    depth.clear();

    std::vector<float> read_depth(data_num, -500.0);
    

    while(infile){
        std::string s;
        if(!getline(infile, s))
            break;
        std::istringstream ss(s);
        std::vector<std::string> record;
        while(ss){
            std::string s;
            if(!getline(ss, s, ','))
                break;
            s.erase(std::remove(s.begin(), s.end(), '\t'), s.end());
            record.push_back(s);
        }
        int data_idx = std::stoi(record[2]);
        if(data_idx==0)
            continue;

        if(read_depth[data_idx-1]>0)
            continue;

        float cur_depth = std::stod(record[1]);
        if(cur_depth>0 && cur_depth<0.2){
            cur_depth = 0.5;
        }

        read_depth[data_idx-1] = cur_depth*1000.0; //[mm]
    }

    for(int i=0; i<data_num; ++i) {
        if(read_depth[i]<0) {
            read_depth[i] = read_depth[i-1];
        }
    }


    depth = read_depth;
}