
#ifndef WORLD_H
#define WORLD_H

#include"Engine.h"
#include"NewtonRigid.h"
#include"StressFEM.h"
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
	Engine* PhyEngine;
	GraphicsEngine* GraEngine;

public:
	~World();
	World();
	World(string inputpath, string outputpath, float runningtime, int step, string mode);
	void LoadingWorld();
	void init();
	void PhysicsRender(float time, int seq);
	void Physics_run();
	void Graphics_run();
	void run();

};

#endif

