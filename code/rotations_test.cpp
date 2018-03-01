//============================================================================//
//                                Rotations Testing
//============================================================================//
/*
  This module tests the functions from rotations.cpp
*/

#include <iostream>
#include "rotations.h"
#include "constants.h"
using namespace constants;
using namespace std;

int main()
{
    // Test printMatrix() function
    matrix_t test_mat = {{1,2,3},{4,5,6},{7,8,9}};
    printMatrix(test_mat);
    std::cout << std::endl;

    // Test rotX() function
    std::cout << "Should be identity matrix\n";
    matrix_t rotx = rotX(0);
    printMatrix(rotx);
    std::cout << std::endl;

    // Test rotY() function
    std::cout << "Should be identiy matrix\n";
    matrix_t roty = rotY(0);
    printMatrix(roty);
    std::cout << std::endl;

    // Test rotZ() function
    std::cout << "Should be identiy matrix\n";
    matrix_t rotz = rotZ(0);
    printMatrix(rotz);
    std::cout << std::endl;

    // // Test matMult()
    // // Should produce error that inner dimensions do not match
    // matrix_t test_mat2 = {{1,2},{3,4}};
    // matrix_t res = matMult(test_mat, test_mat2);

    // Test matMult()
    // testMat^2
    matrix_t res = matMult(test_mat, test_mat);
    printMatrix(res);
    std::cout << std::endl;

    // Rotate test matrix 90 degrees about X axis
    std::cout << "Rotate 90 degrees about X axis\n";
    rotx = rotX(PI/2);
    cout << "Original:\n";
    printMatrix(test_mat);
    cout << "\n";
    cout << "Rotated 90 degrees:\n";
    printMatrix(matMult(rotx, test_mat));
    cout << endl;


    return 0;
}
