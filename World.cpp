#include "World.h"

World::World()
{
}

World::World(string filepath)
{
	configfilepath = filepath;
}

void World::LoadingWorld()
{
	//loading objs
	currentconfig = string("bin/currentfig");
}

void World::PhysicsRender()
{
}

void World::GraphicsRender()
{
}

void World::outputconfigfile()
{
}
