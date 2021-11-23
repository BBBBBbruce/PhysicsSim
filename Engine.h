
#ifndef ENGINE_H
#define ENGINE_H

#include"ExternalLib/pugixml/pugixml.hpp"
#include"StaticObj.h"
#include"DynamicObj.h"
#include <vector>

using namespace std;
class Engine
{
private: 
	vector<StaticObj>StaticVec;
	vector<DynamicObj>DynamicVec;
	string Configfile;

public:
	Engine();
	Engine(string filepath);
	void ParseConfigFile(string filepath);

};

#endif

