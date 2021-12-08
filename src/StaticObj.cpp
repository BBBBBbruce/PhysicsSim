#include "StaticObj.h"

StaticObj::StaticObj()
{
}

StaticObj::StaticObj(string n, 
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
	std::string path
)
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
	loadingpath = path;
}


void StaticObj::displayinfo()
{
	cout << endl;
	cout << "name: " << name << endl;
	//cout << "path: " << loadingpath << endl;
	//cout << "position: " << glm::to_string(position) << endl;
	//cout << "scale: " << glm::to_string(scale) << endl;
	//cout << "rotation: " << glm::to_string(rotation) << endl;
	cout << endl;
}

string StaticObj::get_loadingpath()
{
	return loadingpath;
}

json StaticObj::tojson()
{	
	json j;
	j["type"] = "static";
	//j["path"] = loadingpath;
	//j["position"] = { position.x(),position.y(),position.z() };
	//j["rotation"] = { rotation.x(), rotation.y(), rotation.z() };
	//j["scale"] = { scale.x(),scale.y(),scale.z() };
	return j;
	
}

