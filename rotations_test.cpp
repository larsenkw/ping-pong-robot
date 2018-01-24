#include <iostream>
#include "rotations.h"


int main()
{
    // Test Rot X function
    matrix_t testMat = {{1,2,3},{4,5,6},{7,8,9}};
    testMat = rotX(0);
    printMatrix(testMat);

    return 0;
}
