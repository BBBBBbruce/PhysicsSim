
#include"utiles.h"
#include"Engine.h"

#include"World.h"
#include"test.h"
#include"NewtonRigid.h"

#include "GraphicsEngine.h"

#include"json.hpp"
#include<filesystem>
//#include<CGAL/Simple_cartesian.h>

namespace fs = std::filesystem;

//string testfile = "test.json";
using json = nlohmann::json;
using namespace std;
int main(int argc, char* argv[])
{
    std::cout << "Simulation Starting\n";
    std::cout << "===================\n";
    std::cout << "\n";
    //-i configs/object.json -t 0.01 -n 200 -o bin\out\Project  -m FEM -r 0
    string InputScene = argv[2];
    float runningtime = atof(argv[4]);
    int step = atoi(argv[6]);
    string project_folder = argv[8];
    string sim_mode = argv[10];
    int render = atoi(argv[12]);
    


    //time_t start_time = 1639740809;
    time_t start_time = time(0);
    //cout << start_time << endl;
    project_folder += to_string(start_time) + "\\";

    _mkdir(project_folder.c_str());

    //================fully-auto ==================
    /*World* wrld = new World(InputScene, project_folder, runningtime, step);
    wrld->init();
    wrld->Physics_run();
    wrld->Graphics_run();
    //wrld->run();
    delete wrld;*/
    //================fully-auto ==================
    //============ dynamic method ======================
    World* wrld = new World(InputScene, project_folder, runningtime, step, sim_mode);
    wrld->init();
    wrld->Physics_run();

    delete wrld;
    if (render) {
        GraphicsEngine GraphicsRenderer;
        GraphicsRenderer.run(project_folder);
    }

    //============ dynamic method ====================

    //============ old version ====================
    /*World testwrld(InputScene, project_folder);
 
    World testwrld(InputScene, project_folder);
    testwrld.LoadingWorld();
    testwrld.InitConfigs();
    auto seq = 1;

    for (auto i = 0; i < step; i++) {
        testwrld.PhysicsRender(runningtime, seq);
        seq++; 
    }
    */
    /*string image_folder = project_folder + "images";
    _mkdir(image_folder.c_str());

    GraphicsEngine GraphicsRenderer;

    short sequence = 0;
    for (const auto& entry : fs::directory_iterator(project_folder)) {
        string scenefolder = entry.path().string();
        cout << "loading scene: " << scenefolder << endl;
        GraphicsRenderer.load_scene(scenefolder);
        GraphicsRenderer.save_scene(image_folder,sequence);
        GraphicsRenderer.reset();
        sequence++;
    }*/

    //============ old version ====================

    std::cout << "\n";
    std::cout << "===================\n";
    std::cout << "exit\n";
    std::cout << "\n";
    return 0;
}

