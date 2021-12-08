

#include <iostream>
#include <fstream>
#include <ostream>

#include"Engine.h"

#include"World.h"
#include"test.h"

#include"json.hpp"


//string testfile = "test.json";
using json = nlohmann::json;
using namespace std;

int main()

{
    std::cout << "Simulation Starting\n";
    std::cout << "===================\n";
    std::cout << "\n";

    //PhysicsSim.exe input.json -t 0.1 -n 50 -o bin/output.json
    string InputScene = "configs/object.json";
    string outjson = "bin/out/";
    float runningtime = 0.1;
    int step = 1;
    time_t start_time;
    time_t pre_time;

    World testwrld(InputScene,outjson);
    testwrld.LoadingWorld();
    start_time = testwrld.InitConfigs();
    pre_time = start_time;
    for (auto i = 0; i < step; i++) {
        pre_time = testwrld.PhysicsRender(runningtime, pre_time);
        //testwrld.outputconfigfile();
    }
    

    std::cout << "\n";
    std::cout << "===================\n";
    std::cout << "exit\n";
    std::cout << "\n";
    return 0;
}

