//============================================================================//
//                                Rotations Module
//============================================================================//
/*
  This module contains functions for performing rotations for robotics.  This includes the single rotation in the X, Y, and Z axes, ZYZ and RPY rotations sets and representation in Quaternion and Twist space.
*/

#include <math.h> // for cos(), sin(), atan2()
#include <vector>
#include <iostream>
#include <stdlib.h> // for exit()
#include "constants.h" // for PI

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

// Printing function for debugging vectors
void printVector(std::vector<double> vec)
{
    int num_elem = vec.size();
    for (int i = 0; i < num_elem; ++i){
        std::cout << vec.at(i) << "\n";
    }
}

// Unified printing function
void printVals(matrix_t matrix){
    printMatrix(matrix);
}

void printVals(std::vector<double> vec){
    printVector(vec);
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

// Convert rotation matrix to Roll, Pitch, and Yaw angles
// Input: rotation matrix (3x3)
// Output: (6x1) vector with elements
//      Angle between -pi/2 and pi/2
//      0: roll1, 1: pitch1, 2: yaw1
//      Angle between pi/2 and 3pi/2
//      3: roll2, 4: pitch2, 5: yaw2
std::vector<double> rot2RPY(matrix_t R)
{
    std::vector<double> angles(6);

    double roll1, roll2, pitch1, pitch2, yaw1, yaw2;

    // Case where pitch is within range (-pi/2,pi/2)
    roll1 = atan2(R.at(1).at(0), R.at(0).at(0));
    pitch1 = atan2(-R.at(2).at(0), sqrt(pow(R.at(2).at(1),2) + pow(R.at(2).at(2),2)));
    yaw1 = atan2(R.at(2).at(1), R.at(2).at(2));

    // Case where pitch is within range (pi/2,3pi/2)
    roll2 = atan2(-R.at(1).at(0), -R.at(0).at(0));
    pitch2 = atan2(-R.at(2).at(0), -sqrt(pow(R.at(2).at(1),2) + pow(R.at(2).at(2),2)));
    yaw2 = atan2(-R.at(2).at(1), -R.at(2).at(2));

    if (pitch1 == constants::PI/2){
        roll1 = atan2(R.at(1).at(2), R.at(0).at(2));
        roll2 = atan2(R.at(1).at(2), R.at(0).at(2));
        yaw1 = 0;
        yaw2 = 0;
    }

    angles.at(0) = roll1;
    angles.at(1) = pitch1;
    angles.at(2) = yaw1;
    angles.at(3) = roll2;
    angles.at(4) = pitch2;
    angles.at(5) = yaw2;

    return angles;
}

// Convert rotation matrix to Z0,Y,Z1 Euler angles
// Input: rotation matrix (3x3)
// Output: (3x1) vector with elements
//      0: z0, 1: y, 2: z1
std::vector<double> rot2ZYZ(matrix_t R)
{
    std::vector<double> angles(3);

    double z0, y, z1;

    // Determine y
    y = acos(R.at(2).at(2));

    // If y = 0
    if (y == 0){
        z0 = atan2(R.at(1).at(0), R.at(1).at(1));
        z1 = 0;
    }

    // if y = pi
    else if (y == -constants::PI){
        z0 = -atan2(R.at(1).at(0), R.at(1).at(1));
        z1 = 0;
    }

    else {
        z0 = atan2(R.at(1).at(2), R.at(0).at(2));
        z1 = atan2(R.at(2).at(1), -R.at(2).at(0));
    }

    angles.at(0) = z0;
    angles.at(1) = y;
    angles.at(2) = z1;

    return angles;
}
