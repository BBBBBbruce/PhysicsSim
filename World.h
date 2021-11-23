
#ifndef WORLD_H
#define WORLD_H

#include"NewtonRigid.h"
#include"ExternalLib/pugixml/pugixml.hpp"


class World
{
private:
	string configfilepath;
	pugi::xml_parse_result currentconfig;
	pugi::xml_parse_result outputconfig;
	string PhysicsEngine;

	



public:

	World();
	World(string filepath);
	void LoadingWorld();
	void PhysicsRender();
	void GraphicsRender();
	void outputconfigfile();

};

#endif

