

#include <iostream>

#include"Engine.h"
#include<assimp/Importer.hpp>
#include<assimp/scene.h>
#include<assimp/postprocess.h>
#include"World.h"
#include"test.h"


string setup = "objects.xml";

int main()

{
    std::cout << "Simulation Starting\n";
    std::cout << "===================\n";
    std::cout << "\n";

    test(setup);
    World testwrld(setup);
    testwrld.LoadingWorld();

    std::cout << "\n";
    std::cout << "===================\n";
    std::cout << "exit\n";
    std::cout << "\n";
    return 0;
}

