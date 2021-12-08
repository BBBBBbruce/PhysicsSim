
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
	

public:
	Engine();
	Engine(string filepath);
	//void ParseConfigFile(string filepath);

};

#endif

