//
//  forceCalculation.metal
//  Lattice Boltzman
//
//  Created by Charlie Close on 18/12/2024.
//

#include <metal_stdlib>
#include "./Helpers/Interpolate/Interpolate.h"
using namespace metal;


kernel void forceCalculation(device float3* vertices,
                             device float* forces,
                             device const float3* velocities,
                             device const float* densities,
                             device const int3* latticeSize,
                             uint index [[thread_position_in_grid]])
{
    float3 position = vertices[index];
    float force = forces[index];
    
    float4 velocityAndDensity = interpolate(position, *latticeSize, velocities, densities);
    
    float3 velocity = float3(velocityAndDensity);
    float density = velocityAndDensity[3];
    float momentum = density * length(velocity);
    force = force * 0.8 + density * 0.2;
    forces[index] = force;
}
