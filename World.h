#pragma once
#ifndef WORLD_H
#define WORLD_H

#include"NewtonRigid.h"


class World
{
private:
	string configfilepath;
	string currentconfig;
	string outputconfig;
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

