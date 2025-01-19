//
//  Interpolate.h
//  Lattice Boltzman
//
//  Created by Charlie Close on 18/12/2024.
//

#ifndef Interpolate_h
#define Interpolate_h

#include <metal_stdlib>
using namespace metal;

float4 interpolate(float3 position, int3 latticeSize, device const float3* velocities, device const float* densities);

#endif /* Interpolate_h */
