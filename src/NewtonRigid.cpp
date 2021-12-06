#include "NewtonRigid.h"
#include <glm/gtc/type_ptr.hpp>
using namespace std;

glm::vec3 gravity(0.0, 0.0, -9.8);

NewtonRigid::NewtonRigid()
{
    vector<StaticObj>StaticVec;
    vector<DynamicObj>DynamicVec;
    
}

void NewtonRigid::run(float time)
{
    for (auto i = 0; i < DynamicVec.size(); i++) {
        glm::vec3 v = gravity * time;
        glm::vec3 d = glm::vec3(0.5 * time * time) * gravity;
        DynamicVec[i].updatestate(d, v);
        //update position
    }
}

bool NewtonRigid::collision_detection()
{
    return false;
}

void NewtonRigid::ParseWorld(json objectlist)
{
    //cout << "bbb" << endl;
    for (auto it = objectlist.begin(); it != objectlist.end(); ++it)
    {
        //cout << "aaa" << endl;
        
        if (it.value()["type"] == "dynamic") {
            //cout << it.value()["position"].get<> << endl;
            DynamicObj dtmp(
                it.key(),
                it.value()["path"],
                glm::vec3({ it.value()["position"][0],it.value()["position"][1], it.value()["position"][2] }),
                glm::vec3({ it.value()["scale"][0],it.value()["scale"][1], it.value()["scale"][2] }),
                glm::vec3({ it.value()["rotation"][0],it.value()["rotation"][1], it.value()["rotation"][2] }),
                glm::vec3({ it.value()["velocity"][0],it.value()["velocity"][1], it.value()["velocity"][2] }),
                it.value()["mass"]
            );
            DynamicVec.push_back(dtmp);

        }
        else if (it.value()["type"] == "static") {

            StaticObj stmp(
                it.key(),
                it.value()["path"],
                glm::vec3({ it.value()["position"][0],it.value()["position"][1], it.value()["position"][2] }),
                glm::vec3({ it.value()["scale"][0],it.value()["scale"][1], it.value()["scale"][2] }),
                glm::vec3({ it.value()["rotation"][0],it.value()["rotation"][1], it.value()["rotation"][2] })
                
            );
            StaticVec.push_back(stmp);

        }
    }
}

vector<StaticObj> NewtonRigid::getStaticObjs()
{
    return StaticVec;
}

vector<DynamicObj> NewtonRigid::getDynamicObjs()
{
    return DynamicVec;
}

void NewtonRigid::ShowObjectsInfo()
{
    for (auto i = 0; i < DynamicVec.size(); i++)
        DynamicVec[i].displayinfo();
    for (auto i = 0; i < StaticVec.size(); i++)
        StaticVec[i].displayinfo();
}




