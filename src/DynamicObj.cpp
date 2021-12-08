#include "DynamicObj.h"


DynamicObj::DynamicObj()
{
}

DynamicObj::DynamicObj(string n, string path, float m)
{
	name = n;
	loadingpath = path;
	mass = m;
	position = Eigen::Vector3f{ 0.,0.,0. };
	velocity = Eigen::Vector3f{ 0.,0.,0. };
	
}

DynamicObj::DynamicObj(string n, string path, Eigen::Vector3f pos, Eigen::Vector3f sc, Eigen::Vector3f rot, Eigen::Vector3f vel, float m)
{
	name = n;
	loadingpath = path;
	mass = m;
	position = pos;
	scale = sc;
	rotation = rot;
	velocity = vel;
}

Eigen::Vector3f DynamicObj::get_velocity()
{
	return velocity;
}

float DynamicObj::get_mass()
{
	return mass;
}



void DynamicObj::updatestate(Eigen::Vector3f pos, Eigen::Vector3f v)
{
	position += pos;
	velocity += v;
}

void DynamicObj::displayinfo()
{	
	cout << endl;
	cout << "name: " << name << endl;
	cout << "path: " << loadingpath << endl;
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
	j["path"] = loadingpath;
	j["position"] = { position.x(),position.y(),position.z() };
	j["rotation"] = { rotation.x(), rotation.y(), rotation.z() };
	j["scale"] = { scale.x(),scale.y(),scale.z() };
	j["velocity"] = { velocity.x(),velocity.y(),velocity.z() };
	j["mass"] = mass;
	return j;
}
