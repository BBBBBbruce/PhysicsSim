#include "World.h"
#include <sys/stat.h>
#include <fstream>
#pragma warning (disable : 4996)
//#include"ExternalLib/pugixml/pugixml.hpp"

using namespace std;


inline bool checkfile(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}


World::~World()
{
	//delete[] currentconfig;
	//delete[] outputconfig;
}

World::World()
{
}

World::World(string inputpath, string outputpath)
{
	configfilepath = inputpath;
	targetpath = outputpath;
	NewtonRigid PhyEngine;
}

void World::LoadingWorld()
{
	std::ifstream ifs(configfilepath);
	json j;

	try
	{
		j = json::parse(ifs);
	}
	catch (json::parse_error& ex)
	{
		std::cerr << "parse error at byte " << ex.byte << std::endl;
	}

	currentconfig = j["Objects"];
	
}

void World::PhysicsRender(float time)
{
	PhyEngine.ParseWorld(currentconfig);
	PhyEngine.run(time);
}

void World::GraphicsRender()
{
}

void World::outputconfigfile()
{
	json jout;
	vector<StaticObj> svec = PhyEngine.getStaticObjs();
	vector<DynamicObj> dvec = PhyEngine.getDynamicObjs();
	for (auto i = 0; i < svec.size(); i++) {
		jout["Objects"][svec[i].get_name()] = svec[i].tojson();

	}
	for (auto i = 0; i < dvec.size(); i++) {
		jout["Objects"][dvec[i].get_name()] = dvec[i].tojson();

	}

	std::ofstream o(targetpath);
	o << std::setw(4) << jout << std::endl;
}


