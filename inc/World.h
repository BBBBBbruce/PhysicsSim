
#ifndef WORLD_H
#define WORLD_H

#include"NewtonRigid.h"
#include"GraphicsEngine.h"

using json = nlohmann::json;
class World
{

private:
	float runningtime;
	int step;
	string configfilepath;
	string targetpath;
	json currentconfig;
	json outputconfig;
	NewtonRigid* PhyEngine;
	GraphicsEngine* GraEngine;

public:
	~World();
	World();
	World(string inputpath, string outputpath, float runningtime, int step);
	void LoadingWorld();
	void init();
	void PhysicsRender(float time, int seq);
	void InitConfigs();
	void Physics_run();
	void Graphics_run();
	void run();

};

#endif

