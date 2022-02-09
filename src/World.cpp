#include "World.h"
#include <sys/stat.h>
#include <fstream>
#include <ctime>
#include <direct.h>
#include <ctime>
#pragma warning (disable : 4996)

using namespace std;

inline bool checkfile(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

World::~World()
{
	delete PhyEngine;
	delete GraEngine;
}

World::World()
{
	PhyEngine = new Engine();
	GraEngine = new GraphicsEngine();
}

World::World(string inputpath, string outputpath, float runningtime, int step, string mode)
{

	this->runningtime = runningtime;
	this->step = step;
	configfilepath = inputpath;
	targetpath = outputpath;
	GraEngine = new GraphicsEngine();


	if (mode == "Rigid" || mode == "r")
		PhyEngine = new NewtonRigid(targetpath);
	else if (mode == "FEM" || mode == "f") {
		//cout << "FEM engine" << endl;
		PhyEngine = new StressFEM(targetpath); 
	}
	else
		PhyEngine = new Engine();
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

void World::init()
{
	LoadingWorld();
	PhyEngine->InitConfigs(targetpath, currentconfig);
	//InitConfigs();
}

void World::PhysicsRender(float runningtime, int seq)
{	
	make_dir_win(targetpath, seq);
	PhyEngine->load_scene(seq - 1);
	PhyEngine->run(runningtime, seq);
	PhyEngine->save_scene(seq);
	PhyEngine->reset();

}

void World::Physics_run()
{
	short seq = 1;
	for (auto i = 0; i < step; i++) {
		std::cout << "rendering physics seq: " << seq << std::endl;
		clock_t begin = clock();
		PhysicsRender(runningtime, seq);
		clock_t end = clock();
		float elapsed_secs = float(end - begin) / CLOCKS_PER_SEC;
		std::cout << "Finished, time taken: "<< elapsed_secs<< std::endl;
		std::cout << std::endl;
		seq++;
	}
}

namespace fs = std::filesystem;

void World::Graphics_run()
{
	string image_folder = targetpath + "images";
	_mkdir(image_folder.c_str());
	short sequence = 0;
	for (const auto& entry : fs::directory_iterator(targetpath)) {
		string scenefolder = entry.path().string();
		cout << "loading scene: " << scenefolder << endl;
		GraEngine->load_scene(scenefolder);
		GraEngine->save_scene(image_folder, sequence);
		GraEngine->reset();
		sequence++;
	}
}

void World::run()
{
	init();
	Physics_run();
	Graphics_run();
}
