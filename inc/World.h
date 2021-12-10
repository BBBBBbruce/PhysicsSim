
#ifndef WORLD_H
#define WORLD_H

#include"NewtonRigid.h"

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
	void init();
	time_t PhysicsRender(float time, time_t pre_time);
	void GraphicsRender(time_t start_time);
	void outputconfigfile();
	time_t InitConfigs();

};

#endif

