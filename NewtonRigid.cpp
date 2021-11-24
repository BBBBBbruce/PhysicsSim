#include "NewtonRigid.h"

using namespace std;

NewtonRigid::NewtonRigid()
{
    vector<StaticObj>StaticVec;
    vector<DynamicObj>DynamicVec;
    
}

NewtonRigid::NewtonRigid(xml currentconfig)
{
    vector<StaticObj>StaticVec;
    vector<DynamicObj>DynamicVec;
    xml node = currentconfig;
    std::cout << "hello" << std::endl;
}

void NewtonRigid::run(float time)
{
}

bool NewtonRigid::collision_detection()
{
    return false;
}

xml NewtonRigid::ExportWorld()
{

    return xml();
}

void NewtonRigid::ParseWorld()
{
    // parse all the objects and push them into vectors
    cout << "parsing world" << endl;
    cout << node.child("object").attribute("category").value() << endl;



}
