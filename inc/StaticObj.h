
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
    StaticObj(string n, Eigen::MatrixXd pos, Eigen::MatrixXi tet, Eigen::MatrixXi tri, Eigen::MatrixXi tritag, Eigen::MatrixXi tettag, std::vector<std::string> xfields, std::vector<std::string> efields, std::vector<Eigen::MatrixXd> xf, std::vector<Eigen::MatrixXd>trif, std::vector<Eigen::MatrixXd> tetf,string path);
    //StaticObj(string n, string path, Eigen::Vector3f pos, Eigen::Vector3f sc, Eigen::Vector3f rot);
    void displayinfo();
    string get_loadingpath();
    json tojson();

};
#endif

