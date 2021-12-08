#include "DynamicObj.h"


DynamicObj::DynamicObj()
{
}

/*
DynamicObj::DynamicObj(string n, string path, float m)
{
	name = n;
	loadingpath = path;
	mass = m;
	position = Eigen::Vector3f{ 0.,0.,0. };
	velocity = Eigen::Vector3f{ 0.,0.,0. };
	
}*/

DynamicObj::DynamicObj(string n, 
	Eigen::MatrixXd pos, 
	Eigen::MatrixXi tet, 
	Eigen::MatrixXi tri, 
	Eigen::MatrixXi tritag, 
	Eigen::MatrixXi tettag, 
	std::vector<std::string> xfields, 
	std::vector<std::string> efields, 
	std::vector<Eigen::MatrixXd> xf, 
	std::vector<Eigen::MatrixXd> trif, 
	std::vector<Eigen::MatrixXd> tetf, 
	Eigen::MatrixXd vel, 
	float m)
{
	name = n;
	position = pos;
	tetrahedral = tet;
	Tri = tri;
	TriTag = tritag;
	TetTag = tettag;
	XFields = xfields;
	EFields = efields;
	XF = xf;
	TriF = trif;
	TetF = tetf;
	velocity = vel;
	mass = m;
}

Eigen::MatrixXd DynamicObj::get_velocity()
{
	return velocity;
}

float DynamicObj::get_mass()
{
	return mass;
}
/*
void DynamicObj::updatestate(Eigen::Vector3f pos, Eigen::Vector3f v)
{
	position += pos;
	velocity += v;
}*/

void DynamicObj::displayinfo()
{	
	cout << endl;
	cout << "name: " << name << endl;
	//cout << "path: " << loadingpath << endl;
	//cout << "position: " << glm::to_string(position) << endl;
	//cout << "scale: " << glm::to_string(scale) << endl;
	//cout << "rotation: " << glm::to_string(rotation) << endl;
	//cout << "velocity: " << glm::to_string(velocity) << endl;
	cout << "mass: " << mass << endl;
	cout << endl;
}

json DynamicObj::tojson()
{
	json j;
	j["type"] = "dynamic";
	//j["path"] = loadingpath;
	//j["position"] = { position.x(),position.y(),position.z() };
	//j["rotation"] = { rotation.x(), rotation.y(), rotation.z() };
	//j["scale"] = { scale.x(),scale.y(),scale.z() };
	//j["velocity"] = { velocity.x(),velocity.y(),velocity.z() };
	j["mass"] = mass;
	return j;
}

void DynamicObj::writemsh(string p, string v)
{
	igl::writeMSH(p, position, Tri, tetrahedral, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
	igl::writeMSH(v, velocity, Tri, tetrahedral, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
}
