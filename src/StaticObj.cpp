#include "StaticObj.h"

StaticObj::StaticObj()
{
}

StaticObj::StaticObj(string n, string path)
{
	name = n;
	loadingpath = path;
}

StaticObj::StaticObj(string n, string path, glm::vec3 pos)
{
	name = n;
	loadingpath = path;
	position = pos;
}

void StaticObj::initialise()
{

}

void StaticObj::displayinfo()
{
	cout << "name: " << name << endl;
	cout << "path: " << loadingpath << endl;
	cout << endl;
}

json StaticObj::tojson()
{	
	json j;
	j["type"] = "static";
	j["path"] = loadingpath;
	j["position.x"] = position.x;
	j["position.y"] = position.y;
	j["position.z"] = position.z;
	return j;
}
