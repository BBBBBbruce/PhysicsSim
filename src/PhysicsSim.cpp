

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

string jsonfile = "object.json";
string outjson = "output.json";
//string testfile = "test.json";
using json = nlohmann::json;
using namespace std;

int main()

{
    std::cout << "Simulation Starting\n";
    std::cout << "===================\n";
    std::cout << "\n";

    string meshpath = "assets/table.msh";
    render(meshpath);
    /*
    std::ifstream ifs(jsonfile);
    cout << "hello" << endl;
    json j;

    try
    {
        j = json::parse(ifs);
    }
    catch (json::parse_error& ex)
    {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
    }

    json objectlist = j["Objects"];

    cout << "maybe here" << endl;
    vector<StaticObj> svec;
    vector<DynamicObj> dvec;
    cout << "not here" << endl;

    for (auto it = objectlist.begin(); it != objectlist.end(); ++it)
    {

        std::cout << it.key() << " : " << it.value() << std::endl;
        if (it.value()["type"] == "dynamic") {
            cout << "ddd\n";
            DynamicObj dtmp(
                it.key(),
                it.value()["path"],
                glm::vec3{ it.value()["position.x"],it.value()["position.y"],it.value()["position.z"] },
                glm::vec3{ it.value()["velocity.x"],it.value()["velocity.y"],it.value()["velocity.z"] },
                it.value()["mass"]
            );
            dvec.push_back(dtmp);
            cout << "pushed dynamic" << endl;

        }
        else if (it.value()["type"] == "static"){
            cout << "sss\n";
            StaticObj stmp(
                it.key(),
                it.value()["path"],
                glm::vec3{ it.value()["position.x"],it.value()["position.y"],it.value()["position.z"] }
            );
            svec.push_back(stmp);
            cout << "pushed static" << endl;

        }
    }


    //+++++++++++++++++++++++++++++++++++
    // save for physics sim 
    float timeinterval = 1;//seconds
    glm::vec3 gravity(0,0,-9.8);
    for (auto i = 0; i < dvec.size(); i++) {
        glm::vec3 v = gravity * timeinterval;
        glm::vec3 d = glm::vec3(0.5 * timeinterval * timeinterval)* gravity;
        dvec[i].updatestate(d,v);
        //update position
    }


 
    //+++++++++++++++++++++++++++++++++++

    json jout;
    for (auto i = 0; i < svec.size(); i++) {
        jout["Objects"][svec[i].get_name()] = svec[i].tojson();

    }
    for (auto i = 0; i < dvec.size(); i++) {
        jout["Objects"][dvec[i].get_name()] = dvec[i].tojson();

    }

    std::ofstream o(outjson);
    o << std::setw(4) << jout << std::endl;
    
    */
    // cout << svec.size() << " " << dvec.size() << endl;
    //svec[0].displayinfo();


    //test(setup);
    float runningtime = 1.0;
    World testwrld(jsonfile,outjson);
    testwrld.LoadingWorld();
    testwrld.PhysicsRender(runningtime);
    testwrld.outputconfigfile();

    std::cout << "\n";
    std::cout << "===================\n";
    std::cout << "exit\n";
    std::cout << "\n";
    return 0;
}

