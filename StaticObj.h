
#ifndef STATICOBJ_H
#define STATICOBJ_H

#include "Objects.h"
class StaticObj :
    public Objects
{
public:
    StaticObj(string n, string path);
    StaticObj(string n, string path, coords pos);
    void initialise();
};
#endif

