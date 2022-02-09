#pragma once

#include "Collision.h"

using namespace std;
using namespace Eigen;

tuple<bool, Vector3f> CD_bowl(MatrixXf vertices) {

    // calculate for 2D case: xy plane, z stays constant
    // bowl: 
    // -4 <= x < -2, y = -4x - 8
    // -2 <= x <  2, y =  0
    //  2 <= x <  4, y =  4x - 8
    float distance = 0;
    Vector3f contact_p = { 0.0,0.0,0.0 };
    bool collide = false;
    for (auto i = 0; i < vertices.rows(); i++) {
        float x = vertices(i, 0);// x
        float y = vertices(i, 1);// y
        //float z = vertices(i, 2);// z

        if (-4 <= x && x < -2 && y <= 8) {
            if (4 * x + y + 8 <= 0 && abs(4 * x + y + 8) / sqrt(17) > distance) {//land on LHS
                collide = true;
                distance = abs(4 * x + y + 8) / sqrt(17);
                contact_p = vertices.row(i);
            }
        }
        else if (-2 <= x && x < 2) {
            if (y <= 0 && abs(y) > distance) {
                collide = true;
                distance = abs(y);
                contact_p = vertices.row(i);
            }
        }
        else if (2 <= x && x < 4 && y <= 8) {
            if (4 * x - y - 8 >= 0 && abs(4 * x - y - 8) / sqrt(17) > distance) {//land on RHS
                collide = true;
                distance = abs(4 * x - y - 8) / sqrt(17);
                contact_p = vertices.row(i);
            }
        }
    }

    return{ collide, contact_p };

}

tuple<bool, Vector3f> CD_table(MatrixXf vertices) {

    // calculate for 2D case: xy plane, z stays constant
    // bowl: 
    // -4 <= x < -2, y = -4x - 8
    // -2 <= x <  2, y =  0
    //  2 <= x <  4, y =  4x - 8
    float distance = 0;
    Vector3f contact_p = { 0.0,0.0,0.0 };
    bool collide = false;
    for (auto i = 0; i < vertices.rows(); i++) {
        float x = vertices(i, 0);// x
        float y = vertices(i, 1);// y
        //float z = vertices(i, 2);// z

        if (y <= 0 && abs(y) > distance) {
            collide = true;
            distance = abs(y);
            contact_p = vertices.row(i);
        }

    }

    return{ collide, contact_p };

}

tuple<bool, Vector3f> CD_bowl_wide(MatrixXf vertices)
{
    // calculate for 2D case: xy plane, z stays constant
// bowl: 
// -4 <= x < -2, y = -4x - 8
// -2 <= x <  2, y =  0
//  2 <= x <  4, y =  4x - 8
    float distance = 0;
    Vector3f contact_p = { 0.0,0.0,0.0 };
    bool collide = false;
    for (auto i = 0; i < vertices.rows(); i++) {
        float x = vertices(i, 0);// x
        float y = vertices(i, 1);// y
        //float z = vertices(i, 2);// z

        if (-9 <= x && x < -3 && y <= 6) {
            if ( x + y + 3 <= 0 && abs(x + y + 3) / sqrt(2) > distance) {//land on LHS
                collide = true;
                distance = abs(x + y + 3) / sqrt(2);
                contact_p = vertices.row(i);
            }
        }
        else if (-3 <= x && x < 3) {
            if (y <= 0 && abs(y) > distance) {
                collide = true;
                distance = abs(y);
                contact_p = vertices.row(i);
            }
        }
        else if (3 <= x && x < 9 && y <= 6) {
            if (x - y - 3 >= 0 && abs(x - y - 3) / sqrt(2) > distance) {//land on RHS
                collide = true;
                distance = abs(x - y - 3) / sqrt(2);
                contact_p = vertices.row(i);
            }
        }
    }

    return{ collide, contact_p };
}

tuple<bool, Vector3f> CD_vshape(MatrixXf vertices)
{
    // calculate for 2D case: xy plane, z stays constant
// bowl: 
// -4 <= x < -2, y = -4x - 8
// -2 <= x <  2, y =  0
//  2 <= x <  4, y =  4x - 8
    float distance = 0;
    Vector3f contact_p = { 0.0,0.0,0.0 };
    bool collide = false;
    for (auto i = 0; i < vertices.rows(); i++) {
        float x = vertices(i, 0);// x
        float y = vertices(i, 1);// y
        //float z = vertices(i, 2);// z

        if (-4 <= x && x < -2 && y <= 8) {
            if (4 * x + y + 8 <= 0 && abs(4 * x + y + 8) / sqrt(17) > distance) {//land on LHS
                collide = true;
                distance = abs(4 * x + y + 8) / sqrt(17);
                contact_p = vertices.row(i);
            }
        }
        else if (-2 <= x && x < 2) {
            if (y <= 0 && abs(y) > distance) {
                collide = true;
                distance = abs(y);
                contact_p = vertices.row(i);
            }
        }
        else if (2 <= x && x < 4 && y <= 8) {
            if (4 * x - y - 8 >= 0 && abs(4 * x - y - 8) / sqrt(17) > distance) {//land on RHS
                collide = true;
                distance = abs(4 * x - y - 8) / sqrt(17);
                contact_p = vertices.row(i);
            }
        }
    }

    return{ collide, contact_p };
}

tuple<bool, vector<int>> CD_table_FEM(MatrixXf vertices)
{
    vector<int> contact_points;
    float distance = 0;
    //Vector3f contact_p = { 0.0,0.0,0.0 };
    bool collide = false;
    for (auto i = 0; i < vertices.rows(); i++) {
        float y = vertices(i, 1);// y
        if (y <= 0 ) {
            collide = true;
            //contact_p = vertices.row(i);
            contact_points.push_back(i);
        }

    }

    return{ collide, contact_points };
}
