
#ifndef STATICOBJ_H
#define STATICOBJ_H

#include "Objects.h"
using json = nlohmann::json;

class StaticObj :
    public Objects
{

private: 
    string loadingpath;

public:
    StaticObj();
    StaticObj(string n, Eigen::MatrixXf pos, Eigen::MatrixXi tet, Eigen::MatrixXi tri, Eigen::MatrixXi tritag, Eigen::MatrixXi tettag, std::vector<std::string> xfields, std::vector<std::string> efields, std::vector<Eigen::MatrixXf> xf, std::vector<Eigen::MatrixXf>trif, std::vector<Eigen::MatrixXf> tetf,string path);
    //StaticObj(string n, string path, Eigen::Vector3f pos, Eigen::Vector3f sc, Eigen::Vector3f rot);
    void displayinfo();
    string get_loadingpath();
    json tojson();
    void ToViewer(Eigen::MatrixXf& vertices, Eigen::MatrixXi& faces);
    tuple<Eigen::MatrixXf, Eigen::MatrixXi> Get_ViewMatrix();
};
#endif

