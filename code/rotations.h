#ifndef ROTATIONS_H
#define ROTATIONS_H
#include <vector>

// create matrix type (2D vector)
using matrix_t = std::vector<std::vector<double>>;

//========== Matrix Operations ==========//
// Printing function for debugging matrix calculations
void printMatrix(matrix_t matrix);

// Printing function for debugging vectors
void printVector(std::vector<double> vec);

// Unified printing function
void printVals(matrix_t matrix);
void printVals(std::vector<double> vec);

// Multiply two matrices together
matrix_t matMult(matrix_t matrix1, matrix_t matrix2);

//========== Rotations ==========//
// Rotation around X Axis
// Input: theta in radians
// Output: 3x3 vector of doubles
matrix_t rotX(double theta);

// Rotation around Y Axis
// Input: theta in radians
// Output: 3x3 vector of doubles
matrix_t rotY(double theta);

// Rotation around Z Axis
// Input: theta in radians
// Output: 3x3 vector of doubles
matrix_t rotZ(double theta);

// Convert rotation matrix to Roll, Pitch, and Yaw angles
// Input: rotation matrix (3x3)
// Output: (6x1) vector with elements
//      Angle between -pi/2 and pi/2
//      0: roll1, 1: pitch1, 2: yaw1
//      Angle between pi/2 and 3pi/2
//      3: roll2, 4: pitch2, 5: yaw2
std::vector<double> rot2RPY(matrix_t R);

// Convert rotation matrix to Z0,Y,Z1 Euler angles
// Input: rotation matrix (3x3)
// Output: (3x1) vector with elements
//      0: z0, 1: y, 2: z1
std::vector<double> rot2ZYZ(matrix_t R);

#endif
