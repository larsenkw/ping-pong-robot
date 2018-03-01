#ifndef ROTATIONS_H
#define ROTATIONS_H
#include <vector>

// create matrix type (2D vector)
using matrix_t = std::vector<std::vector<double>>;

//========== Matrix Operations ==========//
// Printing function for debugging matrix calculations
void printMatrix(matrix_t matrix);

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

#endif
