
# Environment Setup
C++ 17\
libigl 2.3.0\
glfw 3.35 WINx64 vc2015\
glad Opengl 3.3 core\
Eigen: check consistence with libigl\
json.h: https://github.com/nlohmann/json

# Mesh Generation
1. create *.obj in Blender.
2. Then using https://github.com/wildmeshing/fTetWild to generate tetrahedral mesh in *.msh

*.msh is format define in Gmsh: https://gmsh.info/doc/texinfo/gmsh.html

# Class Documentation

### Objects

base class of the objects in the world, store information of the objects that spawned

### StaticObj

inherited class from Objects. spawned as world statics. Cannot move or deform. 
Collision detection is allowed.

### DynamicObj

inherited class from Objects. Spawned as world dynamic. 
Has 2 extra properties: velocity and mass (use for physics simulation)
And more properties could be added for deformable simulation

### Engine

Base class of physics engine, process the world and predict the next state

### NewtonRigid

inherited class from Engine. using the basics Newton's laws.

### other simulation

FEM, PBD, PD, etc: adding later

### World

pasring files, calling engines to simulate, export setup for the current state, etc

### visulisation

render the world and display.