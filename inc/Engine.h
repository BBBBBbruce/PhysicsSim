
#ifndef ENGINE_H
#define ENGINE_H


#include"StaticObj.h"
#include"DynamicObj.h"
#include <vector>

using namespace std;
class Engine
{
protected: 
	vector<StaticObj>StaticVec;
	vector<DynamicObj>DynamicVec;
	string tpath;
	float e; // restitution coefficient
	float tc; // collision time
	float u; // friction
	

public:
	Engine();
	Engine(string filepath);
	virtual void load_scene(int pre_seq);
	virtual void run(float time, int seq);
	virtual void save_scene(int seq);
	virtual void InitConfigs(string targetpath, json currentconfig);

	void reset();
	void set_physics_params(float restitution, float collision_t, float friction);

};

#endif

