#include <iostream>
#include <vector>

using matrix_t = std::vector<std::vector<double>>;

int main()
{

    std::vector<std::vector<double>> vector1(4);
    std::cout << vector1.at(0).size() << std::endl;
    std::cout << static_cast<int>(1000.0/10.0) << std::endl;

    return 0;
}
