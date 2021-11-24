
#ifndef WORLD_H
#define WORLD_H

#include"NewtonRigid.h"
#include"ExternalLib/pugixml/pugixml.hpp"

typedef pugi::xml_node xml;

class World
{

private:
	string configfilepath;
	xml currentconfig;
	xml outputconfig;
	NewtonRigid PhyEngine;

public:
	~World();
	World();
	World(string filepath);
	void LoadingWorld();
	void PhysicsRender();
	void GraphicsRender();
	void outputconfigfile();

};

#endif

