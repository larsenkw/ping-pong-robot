#ifndef ROTATIONS_H
#define ROTATIONS_H
#include <vector>

// create matrix type (2D vector)
using matrix_t = std::vector<std::vector<double>>;

//========== Functions ==========//
// Printing function for debugging matrix calculations
void printMatrix(matrix_t matrix);

// Rotation around X Axis
matrix_t rotX(double rad);

#endif
