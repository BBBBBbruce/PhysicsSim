#include "StaticObj.h"

StaticObj::StaticObj(string n, string path)
{
	name = n;
	loadingpath = path;
}

StaticObj::StaticObj(string n, string path, coords pos)
{
	name = n;
	loadingpath = path;
	position = pos;
}

void StaticObj::initialise()
{

}
