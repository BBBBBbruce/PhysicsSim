#include "World.h"
#include <sys/stat.h>

inline bool checkfile(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

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
	std::cout << "loading world" << std::endl;
	if (!checkfile(configfilepath)){
		throw std::runtime_error(std::string("Failed to load setup file "));
	}
	std::cout << "reading config" << std::endl;



	currentconfig = string("bin/currentfig.xml");
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


