#include "DynamicObj.h"


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
