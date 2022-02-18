#include "Engine.h"

Engine::Engine()
{
	//this dont do anything
}

Engine::Engine(string filepath)
{
	//Configfile = filepath;
	//ParseConfigFile(Configfile);
}

void Engine::load_scene(int pre_seq)
{
	// do nothing
}

void Engine::run(float time, int seq)
{
	//do nothing
}

void Engine::save_scene(int seq)
{
	//do nothing
}

void Engine::InitConfigs(string targetpath, json currentconfig, float tc)
{
	//do nothing
}

void Engine::reset()
{
	DynamicVec.clear();
	StaticVec.clear();
}

void Engine::set_physics_params(double restitution, double collision_t, double friction)
{
	e = restitution;
	tc = collision_t;
	u = friction;
}


