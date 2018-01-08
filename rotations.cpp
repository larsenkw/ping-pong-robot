//============================================================================//
//                                Rotations Module
//============================================================================//
/*
  This module contains functions for performing rotations for robotics.  This includes the single rotation in the X, Y, and Z axes, ZYZ and RPY rotations sets and representation in Quaternion and Twist space.
*/

#include <math.h> // for cos() and sin()
#include <vector>
#include <iostream>

// create matrix type (2D vector)
using matrix_t = std::vector<std::vector<double>>;

//========== Functions ==========//
// Printing function for debugging matrix calculations
void printMatrix(matrix_t matrix)
{
    int rows = matrix.size();
    int cols = matrix.at(0).size();
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            if (j == (cols - 1))
            {
                std::cout << matrix.at(i).at(j) << '\n';
            }
            else
            {
                std::cout << matrix.at(i).at(j) << '\t';
            }
        }
    }
}

// Rotation around X Axis
// Input: theta in radians
// Output: 3x3 vector of doubles
matrix_t rotX(double theta)
{
    matrix_t R = {{1,0,0},{0,cos(theta),-sin(theta)},{0,sin(theta),cos(theta)}};
    return R;
}

// Rotation around Y Axis
// Input: theta in radians
// Output: 3x3 vector of doubles
matrix_t rotY(double theta)
{
    matrix_t R = {{cos(theta),0,sin(theta)},{0,1,0},{-sin(theta),0,cos(theta)}};
    return R;
}

// Rotation around Z Axis
// Input: theta in radians
// Output: 3x3 vector of doubles
matrix_t rotZ(double theta)
{
    matrix_t R = {{cos(theta),-sin(theta),0},{sin(theta),cos(theta),0},{0,0,1}};
    return R;
}
