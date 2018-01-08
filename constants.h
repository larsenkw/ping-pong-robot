#ifndef CONSTANTS_H
#define CONSTANTS_H
// Must compile 'constants.cpp' with the inclusion of this file

// namespace for frequently used constant values
namespace Constants
{
    // Mathematical Constants
    extern const double pi;
    extern const double e;

    // Physical Constants
    extern const double gravity; // on earth (m/s^2)
    extern const double c_light; // speed of light (m/s)
    extern const double c_sound; // speed of sound (m/s dry air, 20*C)
    extern const double R_gas; // gas constant (J/(mol*K))
    extern const double avogadro; // (mol^(-1))
}

#endif
