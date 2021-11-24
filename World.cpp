#include "World.h"
#include <sys/stat.h>
//#include"ExternalLib/pugixml/pugixml.hpp"

using namespace std;


inline bool checkfile(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

World::~World()
{
}

World::World()
{
}

World::World(string filepath)
{
	configfilepath = filepath;
	xml currentconfig;
	xml outputconfig;
}

void World::LoadingWorld()
{
	//loading objs
	std::cout << "loading world" << std::endl;
	if (!checkfile(configfilepath)){
		throw std::runtime_error(std::string("Failed to load setup file "));
	}
	std::cout << "reading config" << std::endl;

	pugi::xml_document file;
	pugi::xml_parse_result result;

	result = file.load_file(configfilepath.c_str());
	this->currentconfig = file.child("worldobject");
	//cout << currentconfig.child("object").attribute("category").value() << endl;
}

void World::PhysicsRender()
{
	//add augments
	std::cout << configfilepath << endl;
	NewtonRigid PhyEngine(this->currentconfig);
	std::cout << "hello" << std::endl;
	//while(this->currentconfig!=NULL)
		//cout << currentconfig.child("object").attribute("category").value() << endl;
	//PhyEngine.ParseWorld(currentconfig);
	//PhyEngine.run(1);
	//outputconfig = PhyEngine.ExportWorld();
	//outputconfigfile();
}

void World::GraphicsRender()
{
}

void World::outputconfigfile()
{
	//save tree to xml
	//std::cout << "Saving result: " << outputconfig.save_file("bin/save_file_output.xml") << std::endl;
}


