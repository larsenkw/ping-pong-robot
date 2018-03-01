#ifndef CONSTANTS_H
#define CONSTANTS_H
// Must compile 'constants.cpp' with the inclusion of this file

// namespace for frequently used constant values
namespace constants
{
    // Mathematical Constants
    extern const double PI;
    extern const double E;

    // Physical Constants
    extern const double GRAVITY; // on earth (m/s^2)
    extern const double C_LIGHT; // speed of light (m/s)
    extern const double C_SOUND; // speed of sound (m/s dry air, 20*C)
    extern const double R_GAS; // gas constant (J/(mol*K))
    extern const double AVOGADRO; // (mol^(-1))
}

#endif
