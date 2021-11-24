#include "DynamicObj.h"


DynamicObj::DynamicObj()
{
}

DynamicObj::DynamicObj(string n, string path, float m)
{
	name = n;
	loadingpath = path;
	mass = m;
	position = coords{ 0.,0.,0. };
	velocity = coords{ 0.,0.,0. };
	
}

DynamicObj::DynamicObj(string n, string path, coords pos, coords v, float m)
{
	name = n;
	loadingpath = path;
	mass = m;
	position = pos;
	velocity = v;
}

coords DynamicObj::get_velocity()
{
	return velocity;
}

float DynamicObj::get_mass()
{
	return mass;
}

void DynamicObj::initialise()
{

}

void DynamicObj::updatestate(coords pos, coords v)
{
	position = pos;
	velocity = v;
}

void DynamicObj::displayinfo()
{
	cout << "name: " << name << endl;
	cout << "path: " << loadingpath << endl;
	cout << endl;
}

json DynamicObj::tojson()
{
	json j;
	j["type"] = "dynamic";
	j["path"] = loadingpath;
	j["position.x"] = position.x;
	j["position.y"] = position.y;
	j["position.z"] = position.z;
	j["velocity.x"] = velocity.x;
	j["velocity.y"] = velocity.y;
	j["velocity.z"] = velocity.z;
	j["mass"] = mass;
	return j;
}
