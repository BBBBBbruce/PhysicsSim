
#ifndef DYNAMICOBJ_H
#define DYNAMICOBJ_H

#include "Objects.h"
#include <tuple>


class DynamicObj :
    public Objects
{
private:
    Eigen::MatrixXd velocity;
    float mass;

public:
    DynamicObj();
    //DynamicObj(string n, string path, float m);
    DynamicObj(string n, Eigen::MatrixXd pos, Eigen::MatrixXi tet, Eigen::MatrixXi tri, Eigen::MatrixXi tritag, Eigen::MatrixXi tettag, std::vector<std::string> xfields, std::vector<std::string> efields, std::vector<Eigen::MatrixXd> xf, std::vector<Eigen::MatrixXd>trif, std::vector<Eigen::MatrixXd> tetf, Eigen::MatrixXd vel, float m);
    Eigen::MatrixXd get_velocity();
	float get_mass();
    //void updatestate(Eigen::Vector3f pos, Eigen::Vector3f v);
    void displayinfo();
    json tojson();
    void writemsh(string p, string v);
    void ToViewer(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces);
    tuple<Eigen::MatrixXd, Eigen::MatrixXi> Get_ViewMatrix();
    
    
};

#endif
