#include "NewtonRigid.h"

using namespace std;

//glm::vec3 gravity(0.0, 0.0, -9.8);
Eigen::Vector3f gravity(0.0, 0.0, -9.8);

NewtonRigid::NewtonRigid()
{
    
}

NewtonRigid::NewtonRigid(string t_path)
{
    //cout << targetpath << endl;
    vector<StaticObj>StaticVec;
    vector<DynamicObj>DynamicVec;
    this->tpath = t_path;
    //cout <<"engine: "<< targetpath << endl;
}


bool NewtonRigid::collision_detection()
{
    return false;
}

/*
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
                Eigen::Vector3f({ it.value()["position"][0],it.value()["position"][1], it.value()["position"][2] }),
                Eigen::Vector3f({ it.value()["scale"][0],it.value()["scale"][1], it.value()["scale"][2] }),
                Eigen::Vector3f({ it.value()["rotation"][0],it.value()["rotation"][1], it.value()["rotation"][2] }),
                Eigen::Vector3f({ it.value()["velocity"][0],it.value()["velocity"][1], it.value()["velocity"][2] }),
                it.value()["mass"]
            );
            DynamicVec.push_back(dtmp);

        }
        else if (it.value()["type"] == "static") {

            StaticObj stmp(
                it.key(),
                it.value()["path"],
                Eigen::Vector3f({ it.value()["position"][0],it.value()["position"][1], it.value()["position"][2] }),
                Eigen::Vector3f({ it.value()["scale"][0],it.value()["scale"][1], it.value()["scale"][2] }),
                Eigen::Vector3f({ it.value()["rotation"][0],it.value()["rotation"][1], it.value()["rotation"][2] })
                
            );
            StaticVec.push_back(stmp);

        }
    }
}

*/

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

void NewtonRigid::load_scene(time_t t_stamp)
{   
    //bug cannot read tpath?
    cout << "loading" << tpath << endl;
    string scene_in = tpath + to_string(t_stamp) + "/Scene.json";
    cout << endl;
    cout << scene_in << endl;
    cout << endl;
    std::ifstream ifs(scene_in);
    json j;

    try
    {
        j = json::parse(ifs);
    }
    catch (json::parse_error& ex)
    {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
    }

    for (auto it = j["Objects"].begin(); it != j["Objects"].end(); ++it) {
        if (it.value()["type"] == "dynamic") {
            Eigen::MatrixXd X, V;
            Eigen::MatrixXi Tri, Tet;
            Eigen::VectorXi TriTag, TetTag;
            std::vector<std::string> XFields, EFields;
            std::vector<Eigen::MatrixXd> XF, TriF, TetF;

            igl::readMSH(it.value()["position_path"], X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
            igl::readMSH(it.value()["velocity_path"], V, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
            DynamicObj dtmp(
                it.key(),
                X,Tet,Tri,TriTag,TetTag,XFields, EFields,
                XF,TriF,TetF,
                V,it.value()["mass"]
            );
            DynamicVec.push_back(dtmp);
        }
        else if (it.value()["type"] == "static") {
            Eigen::MatrixXd X;
            Eigen::MatrixXi Tri, Tet;
            Eigen::VectorXi TriTag, TetTag;
            std::vector<std::string> XFields, EFields;
            std::vector<Eigen::MatrixXd> XF, TriF, TetF;

            igl::readMSH(it.value()["position_path"], X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
            StaticObj stmp(
                it.key(),
                X, Tet, Tri, TriTag, TetTag, XFields, EFields,
                XF, TriF, TetF, it.value()["position_path"]
            );
            StaticVec.push_back(stmp);
        }
    }
}

void NewtonRigid::run(float time)
{
    /*
    for (auto i = 0; i < DynamicVec.size(); i++) {
        Eigen::Vector3f v = gravity * time;
        Eigen::Vector3f d = Eigen::Vector3f(0.5 * time * time) * gravity;
        DynamicVec[i].updatestate(d, v);
        //update position
    }*/
}

void NewtonRigid::save_scene(time_t t_stamp)
{   
    json jout;

    for (auto i = 0; i < DynamicVec.size(); i++) {
        string path_p = tpath + to_string(t_stamp) + "/" + DynamicVec[i].name + "_p.msh";
        string path_v = tpath + to_string(t_stamp) + "/" + DynamicVec[i].name + "_v.msh";
        DynamicVec[i].writemsh(path_p, path_v);
        json jtmp;
        jtmp["position_path"] = path_p;
        jtmp["velocity_path"] = path_v;
        jtmp["mass"] = DynamicVec[i].get_mass();
        jtmp["type"] = "dynamic";
        jout["Objects"][DynamicVec[i].name] = jtmp;
    }
    for (auto i = 0; i < StaticVec.size(); i++) {
        json jtmp;
        jtmp["position_path"] = StaticVec[i].get_loadingpath();
        jtmp["type"] = "static";
        jout["Objects"][StaticVec[i].name] = jtmp;
    }

    string path_scene = tpath + to_string(t_stamp) + "/" + "Scene.json";
    std::ofstream o(path_scene);
    o << std::setw(4) << jout << std::endl;

}




