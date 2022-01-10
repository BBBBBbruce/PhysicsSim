#include "NewtonRigid.h"

using namespace std;

//glm::vec3 gravity(0.0, 0.0, -9.8);


Eigen::Vector3f gravity(0.0, -10.0, 0.0);

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


void NewtonRigid::load_scene(int pre_seq)
{   

    string scene_in = tpath + padseq(pre_seq) + "/Scene.json";
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
                double2float(X), Tet, Tri, TriTag, TetTag, XFields, EFields,
                cast2float(XF), cast2float(TriF), cast2float(TetF),
                double2float(V), it.value()["mass"]
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
                double2float(X), Tet, Tri, TriTag, TetTag, XFields, EFields,
                cast2float(XF), cast2float(TriF), cast2float(TetF), it.value()["position_path"]
            );
            StaticVec.push_back(stmp);
        }
    }
}

using namespace Eigen;

bool collision_check(MatrixXf vertices, MatrixXi tets) {


    for (auto i = 0; i < tets.rows(); i++) {
        //inline double vertex1 = vertices(tets(i, 0), 1);
        //inline double vertex2 = vertices(tets(i, 1), 1);
        //inline double vertex3 = vertices(tets(i, 2), 1);
        //inline double vertex4 = vertices(tets(i, 3), 1);


        if (vertices(tets(i, 0), 1) <= 0.0) {
            return true;
        }
        else if (vertices(tets(i, 1), 1) <= 0.0) {
            return true;
        }
        else if (vertices(tets(i, 2), 1) <= 0.0) {
            return true;
        }
        else if (vertices(tets(i, 3), 1) <= 0.0) {
            return true;
        }
    }
    return false;
}

void NewtonRigid::run(float time,int seq)
{
    
    for (auto i = 0; i < DynamicVec.size(); i++) {
        Eigen::Vector3f v = gravity * time;
        Eigen::Vector3f d = 0.5 * time * time * gravity;

        MatrixXf vertices = DynamicVec[i].get_position();
        MatrixXf vel = DynamicVec[i].get_velocity();
        MatrixXi tets = DynamicVec[i].get_tetrahedrons();

        vertices.rowwise() += d.transpose();
        vertices += vel * time;

        //assume dynamic objects wont collide with each other. 
        //check colliding with plane z = 0;
        bool collide = collision_check(vertices, tets);
        DynamicVec[i].updatestate(d, v, collide, time, seq);
        //update position
    }
}

void NewtonRigid::save_scene(int seq)
{   
    json jout;

    for (auto i = 0; i < DynamicVec.size(); i++) {
        string path_p = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_p.msh";
        string path_v = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_v.msh";
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

    string path_scene = tpath + padseq(seq) + "/" + "Scene.json";
    std::ofstream o(path_scene);
    o << std::setw(4) << jout << std::endl;

}

#include<cgal/IO/File_tetgen.h>
void NewtonRigid::create_aabb_tree() {

}

void NewtonRigid::reset()
{
    DynamicVec.clear();
    StaticVec.clear();
}




