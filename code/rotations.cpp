//============================================================================//
//                                Rotations Module
//============================================================================//
/*
  This module contains functions for performing rotations for robotics.  This includes the single rotation in the X, Y, and Z axes, ZYZ and RPY rotations sets and representation in Quaternion and Twist space.
*/

#include <math.h> // for cos() and sin()
#include <vector>
#include <iostream>
#include <stdlib.h> // for exit()

// create matrix type (2D vector)
using matrix_t = std::vector<std::vector<double>>;

//========== Matrix Operations ==========//
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

// Multiply two matrices together
matrix_t matMult(matrix_t matrix1, matrix_t matrix2)
{
    // Check that matrix inner dimensions are appropriate, if not exit and throw
    // an error
    if (matrix1.at(0).size() != matrix2.size())
    {
        std::cerr << "Matrix dimensions do not match for multiplication\n";
        std::cerr << "Should be of the form: (a x n) * (n x b)\n";
        exit(1);
    }

    int r1 = matrix1.size();
    int n = matrix1.at(0).size();
    int c2 = matrix2.at(0).size();

    std::vector<std::vector<double>> result;
    result.resize(r1);

    double element_sum;
    for (int i = 0; i < r1; ++i)
    {
        for (int j = 0; j < c2; ++j)
        {
            element_sum = 0;

            for (int k = 0; k < n; ++k)
            {
                element_sum += matrix1.at(i).at(k)*matrix2.at(k).at(j);
            }

            result.at(i).push_back(element_sum);
        }
    }

    return result;
}

//========== Rotations ==========//
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
