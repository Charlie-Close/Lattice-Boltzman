//
//  Parameters.h
//  Lattice Boltzman
//
//  Created by Charlie Close on 17/12/2024.
//

#ifndef Parameters_h
#define Parameters_h

#ifdef __METAL_VERSION__
using namespace metal;
#include <metal_stdlib>
#define COMPAT_CONST constant
#else
#define COMPAT_CONST const
#endif

// Note, due to the nature of the LBM, increasing resolution also decreases the timestep.
COMPAT_CONST int resolution = 1.f * pow(10.f, 6);
COMPAT_CONST char filename[] = "astronaut.stl";
COMPAT_CONST float padding = 0.3;
// Can increase this when running at low resolution so simulation speed is not limited by frame rate.
COMPAT_CONST uint stepsPerFrame = 2;

// Particle parameters (just for rendering)
COMPAT_CONST int nParticles = 20000;
COMPAT_CONST int streakLength = 50;
COMPAT_CONST bool narrow = false; // Just render a single slice (sometimes easier to see what is going on)
COMPAT_CONST bool particlesFixed = false; // Like strings with one end fixed
COMPAT_CONST float particlesSpeedMult = 1; // In fixed mode, defines how long the strings are
COMPAT_CONST float particleSize = 1; // In fixed mode, defines how long the strings are

// Mesh rotation
COMPAT_CONST float theta = 3.1415 / 2;
COMPAT_CONST float phi = 0;
COMPAT_CONST float psi = 3.1415 / 2;



#endif /* Parameters_h */
