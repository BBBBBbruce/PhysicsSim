
# TODO:
add translation, rotation, enlargement in xml file

# environment setup
C++ 20\
assimp 

# world set up

OpenGL xyz: shading using opengl
up: +y
right: +x
towards user(outwards of screen): +z

Blender xyz: more intuitive
up: +z
right: +x
out of screen: +y

programming and defining using Blender system,
using coordinates translation function in Graphics Renderer class(swap y,z)

# class documentation

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

### GracphisRenderer

render the world and display.