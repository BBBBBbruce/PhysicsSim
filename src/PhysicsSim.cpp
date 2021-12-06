

#include <iostream>
#include <fstream>
#include <ostream>

#include"Engine.h"

#include"World.h"
#include"test.h"

#include"json.hpp"
#include"igl/readOBJ.h"
#include"igl/opengl/glfw/Viewer.h"
#include"visualisation.h"

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
    string outjson = "bin/out/output.json";
    float runningtime = 0.1;
    int step = 10;


    World testwrld(InputScene,outjson);
    testwrld.LoadingWorld();
    testwrld.init();
    for (auto i = 0; i < step; i++) {
        testwrld.PhysicsRender(runningtime);
        testwrld.outputconfigfile();
    }
    

    std::cout << "\n";
    std::cout << "===================\n";
    std::cout << "exit\n";
    std::cout << "\n";
    return 0;
}

