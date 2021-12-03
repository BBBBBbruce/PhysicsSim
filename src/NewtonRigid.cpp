#include "NewtonRigid.h"

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
    for (auto it = objectlist.begin(); it != objectlist.end(); ++it)
    {

        if (it.value()["type"] == "dynamic") {
            DynamicObj dtmp(
                it.key(),
                it.value()["path"],
                glm::vec3{ it.value()["position.x"],it.value()["position.y"],it.value()["position.z"] },
                glm::vec3{ it.value()["velocity.x"],it.value()["velocity.y"],it.value()["velocity.z"] },
                it.value()["mass"]
            );
            DynamicVec.push_back(dtmp);

        }
        else if (it.value()["type"] == "static") {
            StaticObj stmp(
                it.key(),
                it.value()["path"],
                glm::vec3{ it.value()["position.x"],it.value()["position.y"],it.value()["position.z"] }
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


