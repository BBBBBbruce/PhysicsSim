
import numpy as np

g = np.array(0,-9.8,0)
M = 10000
N = 1000
E = 5


def internal_collision():
    return np.array(N,3)

def FEM(tet,x,v,m,delta_t):
    '''
    Assume model has N vertices and M tets
    tet is M*4 matrix
    x is N*3 matrix
    v is also N*3 matrix
    m is M*1 matrix
    f is M*3
    '''
    # initialisation stage
    p_s = x
    _x = np.array(tet.len(), 3,3)
    for i in range(_x.len()):
        _x[i] = np.linalg.inv(np.array(x[tet[i,1]] - x[tet[i,0]],
                                       x[tet[i,2]] - x[tet[i,0]],
                                       x[tet[i,3]] - x[tet[i,0]]))

    # simulation stage
    f_collision = internal_collision(x)
    f = m.any()*g + f_collision #f is N*3 for each vertex
    for i in range(tet.len()):
        p0 = p_s[tet[i,1]] - p_s[tet[i,0]]
        p1 = p_s[tet[i,2]] - p_s[tet[i,0]]
        p2 = p_s[tet[i,3]] - p_s[tet[i,0]]
        p3 = p_s[tet[i,2]] - p_s[tet[i,1]]
        p4 = p_s[tet[i,3]] - p_s[tet[i,1]]
        #p5 = p_s[tet[i,3]] - p_s[tet[i,2]]

        p_c = np.array(p0,p1,p2) * _x[i]
        delta_u = p_c - np.identity(p_c)
        strain = 0.5* (delta_u + np.transpose(delta_u) 
                        + np.transpose(delta_u)*delta_u)
        stress = E*strain

        ff1 = stress*(np.cross(p0,p1))#0,1,2
        ff2 = stress*(np.cross(p0,p2))#0,1,3
        ff3 = stress*(np.cross(p1,p2))#0,2,3
        ff4 = stress*(np.cross(p3,p4))#1,2,3

        f[tet[i,0]] += (ff1+ff2+ff3)/3
        f[tet[i,1]] += (ff1+ff2+ff4)/3
        f[tet[i,2]] += (ff1+ff3+ff4)/3
        f[tet[i,3]] += (ff2+ff3+ff4)/3

    #update:
    v   += delta_t * f / m
    p_s += delta_t * v     




    
    
    


