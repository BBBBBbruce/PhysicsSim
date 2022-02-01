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

void Engine::InitConfigs(string targetpath, json currentconfig)
{
	//do nothing
}

void Engine::reset()
{
	DynamicVec.clear();
	StaticVec.clear();
}

void Engine::set_physics_params(float restitution, float collision_t, float friction)
{
	e = restitution;
	tc = collision_t;
	u = friction;
}


