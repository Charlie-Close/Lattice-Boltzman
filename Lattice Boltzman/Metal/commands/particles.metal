//
//  particles.metal
//  Lattice Boltzman
//
//  Created by Charlie Close on 08/12/2024.
//

#include "../../Parameters/Parameters.h"
#include "./Helpers/Interpolate/Interpolate.h"
#include <metal_stdlib>
using namespace metal;

// Generate a random float in the range [0.0f, 1.0f] using x, y, and z (based on the xor128 algorithm)
float rand(int x, int y, int z)
{
    int seed = x + y * 57 + z * 241;
    seed= (seed<< 13) ^ seed;
    return (( 1.0 - ( (seed * (seed * seed * 15731 + 789221) + 1376312589) & 2147483647) / 1073741824.0f) + 1.0f) / 2.0f;
}

kernel void particles(device float3* positions,
                 device float* particleDensities,
                 device float3* particleVelocities,
                 device float* densities,
                 device float3* velocities,
                 device int3* latticeSize,
                 device int* loopPos,
                 uint index [[thread_position_in_grid]])
{
    if (particlesFixed) {
        // Fixed mode
        float3 position = positions[index * streakLength];
        
        for (int i = 1; i < streakLength; i++) {
            float4 velocityAndDensity = interpolate(position, *latticeSize, velocities, densities);
            
            float3 velocity = float3(velocityAndDensity);
            float density = velocityAndDensity[3];
            // Update particle position
            position += normalize(velocity) * particlesSpeedMult / streakLength;
            
            positions[index * streakLength + i] = position;
            particleDensities[index * streakLength + i] = density;
            particleVelocities[index * streakLength + i] = density;
        }
    } else {
        // Streaming mode
        for (int i = 1; i < streakLength; i++) {
            float3 position = positions[index * streakLength + i];
            float4 velocityAndDensity = interpolate(position, *latticeSize, velocities, densities);
            
            float3 velocity = float3(velocityAndDensity);
            float density = velocityAndDensity[3];
            
            particleDensities[index * streakLength + i] = density;
            particleVelocities[index * streakLength + i] = velocity;
        }
        
        
        float3 position = positions[index * streakLength + *loopPos];
        
        float4 velocityAndDensity = interpolate(position, *latticeSize, velocities, densities);
        
        float3 velocity = float3(velocityAndDensity);
        float density = velocityAndDensity[3];
        // Update particle position
        position += velocity * particlesSpeedMult;
        
        // Break if position goes out of bounds
        if (position.x < 0 || position.x >= (*latticeSize).x ||
            position.y < 0 || position.y >= (*latticeSize).y ||
            position.z < 0 || position.z >= (*latticeSize).z) {
            float x = 0;
            float y = rand(position.y * 100, position.z * 100, position.x * 100) * (*latticeSize).y;
            float z = rand(position.z * 100, position.x * 100, position.y * 100) * (*latticeSize).z;
            if (narrow) {
                z = z * 0.001 + (*latticeSize).z * 0.4995;
            }
            position = { x, y, z };
        }
        
        positions[index * streakLength + ((*loopPos + 1) % (streakLength))] = position;
        particleDensities[index * streakLength + ((*loopPos + 1) % (streakLength))] = density;
        particleVelocities[index * streakLength + ((*loopPos + 1) % (streakLength))] = velocity;
    }
}

// Super small kernel to increment the loop position
kernel void incrementLoopPos(device int* loopPos,
                             uint index [[thread_position_in_grid]]) {
    *loopPos = ((*loopPos) + 1) % streakLength;
}
