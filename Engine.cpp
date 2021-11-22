#include "Engine.h"

Engine::Engine()
{
	//this dont do anything
}

Engine::Engine(string filepath)
{
	Configfile = filepath;
	ParseConfigFile(Configfile);
}

void Engine::ParseConfigFile(string filepath)
{
	//TODO, decide using js or txt
}
