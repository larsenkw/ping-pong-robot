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
    //----- Test printMatrix() function
    matrix_t test_mat = {{1,2,3},{4,5,6},{7,8,9}};
    printMatrix(test_mat);
    std::cout << std::endl;

    //----- Test rotX() function
    std::cout << "Should be identity matrix\n";
    matrix_t rotx = rotX(0);
    printMatrix(rotx);
    std::cout << std::endl;

    //----- Test rotY() function
    std::cout << "Should be identiy matrix\n";
    matrix_t roty = rotY(0);
    printMatrix(roty);
    std::cout << std::endl;

    //----- Test rotZ() function
    std::cout << "Should be identiy matrix\n";
    matrix_t rotz = rotZ(0);
    printMatrix(rotz);
    std::cout << std::endl;

    // //----- Test matMult()
    // // Should produce error that inner dimensions do not match
    // matrix_t test_mat2 = {{1,2},{3,4}};
    // matrix_t res = matMult(test_mat, test_mat2);

    //----- Test matMult()
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

    //----- Test rot2RPY()
    cout << "Testing 'rot2RPY'\n";
    // Identity matrix
    // first set of angles are 0
    // second set of angles are -pi
    std::vector<double> angles1;
    matrix_t I3 = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    angles1 = rot2RPY(I3);
    cout << "First angles: " << endl;
    printVals(angles1);
    cout << endl;

    // Matrix:
    // { 0.83838664, -0.45113927, 0.30591043;
    //   0.45801271,  0.88734270, 0.05336006;
    //  -0.29552021,  0.09537451, 0.95056379}
    // roll1 = 0.5, pitch1 = 0.3, yaw1 = 0.1
    // roll2 = -2.6416, pitch2 = 2.8416, yaw2 = -3.0416
    std::vector<double> angles2;
    matrix_t R2 = {{0.83838664, -0.45113927, 0.30591043},
                   {0.45801271,  0.88734270, 0.05336006},
                   {-0.29552021,  0.09537451, 0.95056379}};
    angles2 = rot2RPY(R2);
    cout << "Second angles: " << endl;
    printVals(angles2);
    cout << endl;

    //----- Test rot2ZYZ()
    cout << "Testing 'rot2ZYZ'\n";
    // Identity matrix
    // angles are all 0
    std::vector<double> angles3;
    angles3 = rot2ZYZ(I3);
    cout << "First angles: \n";
    printVals(angles3);
    cout << endl;

    // Matrix:
    // {0.103846565151668,   0.864780102737098,   0.491295496433882;
    //  0.422918571742548,  -0.485478460963668,   0.765147401234293;
    //  0.900197629735517,   0.128320060202457,  -0.416146836547142}
    // z0 = 1, y = 2, z1 = 3
    std::vector<double> angles4;
    matrix_t R3 = {{0.103846565151668, 0.864780102737098, 0.491295496433882},
                   {0.422918571742548,-0.485478460963668, 0.765147401234293},
                   {0.900197629735517, 0.128320060202457,-0.416146836547142}};
    angles4 = rot2ZYZ(R3);
    cout << "Second angles: \n";
    printVals(angles4);
    cout << endl;

    return 0;
}
