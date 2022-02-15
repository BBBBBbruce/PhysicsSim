
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
	double e; // restitution coefficient
	double tc; // collision time
	double u; // friction
	

public:
	Engine();
	Engine(string filepath);
	virtual void load_scene(int pre_seq);
	virtual void run(float time, int seq);
	virtual void save_scene(int seq);
	virtual void InitConfigs(string targetpath, json currentconfig);

	virtual void reset();
	virtual void set_physics_params(double restitution, double collision_t, double friction);

};

#endif

