

#include <iostream>
#include <fstream>
#include <ostream>

#include"Engine.h"

#include"World.h"
#include"test.h"
#include"NewtonRigid.h"

#include <direct.h>
#include "GraphicsEngine.h"


#include"json.hpp"
#include<filesystem>
#include<CGAL/Simple_cartesian.h>

namespace fs = std::filesystem;


//string testfile = "test.json";
using json = nlohmann::json;
using namespace std;

int main(int argc, char* argv[])

{
    //for (auto i = 0; i < argc; i++)
    //    cout << argv[i] << endl;

    std::cout << "Simulation Starting\n";
    std::cout << "===================\n";
    std::cout << "\n";
    //cout << "argv 0: " << argv[0] << endl;
    //cout << "argv 1: " << argv[1] << endl;
    //cout << "argv 2: " << argv[2] << endl;
    //cout << "argv 3: " << argv[3] << endl;
    //cout << "argv 4: " << argv[4] << endl;
    //cout << "argv 5: " << argv[5] << endl;
    //cout << "argv 6: " << argv[6] << endl;
    //cout << "argv 7: " << argv[7] << endl;
 

    //PhysicsSim.exe input.json -t 0.1 -n 50 -o bin/output.json
    //string InputScene = "configs/object.json";
    //string outjson = "bin/out/Project";
    //float runningtime = 0.1;
    //int step = 10;
    string InputScene = argv[2];
    float runningtime = atof(argv[4]);
    int step = atoi(argv[6]);
    string outjson = argv[8];
    //cout << InputScene << endl;
    //cout << step << endl;
    //cout << runningtime << endl;
    //cout << outjson << endl;

    /*
    time_t start_time = time(0);
    cout << start_time << endl;
    outjson += to_string(start_time)+"/";
    _mkdir(outjson.c_str());
    time_t pre_time;
    World testwrld(InputScene,outjson);
    testwrld.LoadingWorld();
    start_time = testwrld.InitConfigs();
    pre_time = start_time;
    for (auto i = 0; i < step; i++) {
        pre_time = testwrld.PhysicsRender(runningtime, pre_time);
        //testwrld.outputconfigfile();
    }
    testwrld.GraphicsRender(start_time);
    */

    time_t start_time = 1638985169;
    string project_folder = outjson + to_string(start_time);
    string image_folder = project_folder + "\\images";
    _mkdir(image_folder.c_str());

    GraphicsEngine GraphicsRenderer;

    short sequence = 0;
    for (const auto& entry : fs::directory_iterator(project_folder)) {
        string scenefolder = entry.path().string();
        cout << "loading scene: " << scenefolder << endl;
        GraphicsRenderer.load_scene(scenefolder);
        GraphicsRenderer.save_scene(image_folder,sequence);
        sequence++;
        break;
    }


        

    

    

    std::cout << "\n";
    std::cout << "===================\n";
    std::cout << "exit\n";
    std::cout << "\n";
    return 0;
}

