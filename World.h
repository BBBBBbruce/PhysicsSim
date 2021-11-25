
#ifndef WORLD_H
#define WORLD_H

#include"NewtonRigid.h"
#include"ExternalLib/pugixml/pugixml.hpp"

typedef pugi::xml_node xml;
using json = nlohmann::json;
class World
{

private:
	string configfilepath;
	string targetpath;
	json currentconfig;
	json outputconfig;
	NewtonRigid PhyEngine;

public:
	~World();
	World();
	World(string inputpath, string outputpath);
	void LoadingWorld();
	void PhysicsRender(float time);
	void GraphicsRender();
	void outputconfigfile();

};

#endif

