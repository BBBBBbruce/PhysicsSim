#include "StaticObj.h"

StaticObj::StaticObj()
{
}

StaticObj::StaticObj(string n, string path)
{
	name = n;
	loadingpath = path;
}

StaticObj::StaticObj(string n, string path, glm::vec3 pos, glm::vec3 sc, glm::vec3 rot)
{
	name = n;
	loadingpath = path;
	position = pos;
	scale = sc;
	rotation = rot;
}

void StaticObj::initialise()
{

}

void StaticObj::displayinfo()
{
	cout << endl;
	cout << "name: " << name << endl;
	cout << "path: " << loadingpath << endl;
	cout << "position: " << glm::to_string(position) << endl;
	cout << "scale: " << glm::to_string(scale) << endl;
	cout << "rotation: " << glm::to_string(rotation) << endl;
	cout << endl;
}

json StaticObj::tojson()
{	
	json j;
	j["type"] = "static";
	j["path"] = loadingpath;
	j["position"] = { position.x,position.y,position.z };
	j["rotation"] = { rotation.x, rotation.y, rotation.z };
	j["scale"] = { scale.x,scale.y,scale.z };
	return j;
}
