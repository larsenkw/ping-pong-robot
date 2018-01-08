/*
    This module contains a series of functions that perform calculations
    required for robotics simulation.
*/

#include <cmath>
#include <iostream>
#include <vector>
using namespace std;

// ROTATIONS
// rotX, rotY, rotZ: return the rotation about the indicated axis by a
// certain angle provided in radians
//
//  [R] = rotX(theta)  returns a 3x3 array for a rotation matrix about the X
//  axis of theta radians.
//  [R] = rotY(theta)  returns a 3x3 array for a rotation matrix about the Y
//  axis of theta radians.
//  [R] = rotZ(theta)  returns a 3x3 array for a rotation matrix about the Z
//  axis of theta radians.

vector<vector<double>> rotX(double theta) {
    vector<vector<double>> R(3, vector<double>(3));
    R[0][0] = 1;
    R[0][1] = 0;

    R[1][0] = 0;
    R[1][1] = cos(theta);

    return R;
}

int main() {
    double theta = 3.141592654;
    vector<vector<double>> R;
    R = rotX(theta);
    cout << "The value of X rotation matrix: \n" << endl;
    cout << R[1][1] << endl;

    return 0;
}
