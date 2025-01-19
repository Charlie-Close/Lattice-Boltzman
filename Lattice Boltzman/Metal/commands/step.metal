//
//  step.metal
//  Lattice Boltzman
//
//  Created by Charlie Close on 05/12/2024.
//

#include <metal_stdlib>
#include "../../Parameters/SimulationConstants.h"
using namespace metal;

// Two methods of streaming - back and forwards streaming.
//
// Forwards streaming uses the data at index i to update the surrounding lattice points
// Back streaming uses the data at the surrounding lattice points to update the data at index i
// Mathematically, these are equivalent.
//
// Forwards streaming results in reading data from just index i, but writing data to scattered indices.
// Back streaming results in reading data from scattered indices and writing data to just index i.
// Scattered write calls are very slow (and are the bottle neck of this simulation e.g. 50 million lattice
// points requires writing 1.4 billion floats every frame). Scattered read calls aren't as bad. Therefore
// backstreaming is better
//
// TODO: could try to implement swizzling so read calls are not as scattered
void backStreaming(float f[ni], device const float* currentF, int f_start, int3 position, int3 latticeSize) {
    for (int i = 0; i < ni; i++) {
        int3 targetPos = position - e[i];
        
        // Repeated boundary condition
        if (targetPos.x < 0 || targetPos.x >= latticeSize.x ||
            targetPos.y < 0 || targetPos.y >= latticeSize.y ||
            targetPos.z < 0 || targetPos.z >= latticeSize.z) {
            targetPos.x = (targetPos.x + latticeSize.x) % latticeSize.x;
            targetPos.y = (targetPos.y + latticeSize.y) % latticeSize.y;
            targetPos.z = (targetPos.z + latticeSize.z) % latticeSize.z;
        }
        
        // Compute the linear index of the target position
        int targetIndex = (targetPos.x + targetPos.y * latticeSize.x + targetPos.z * latticeSize.x * latticeSize.y) * ni;
                
        f[i] = currentF[targetIndex + i];
    }
}

kernel void step(device const float* currentF,
                 device float* nextF,
                 device float* densities,
                 device const int3* positions,
                 device float3* velocities,
                 device const bool* isFixedArr,
                 device const bool* isWallArr,
                 device const bool* isInsideArr,
                 device const int3* latticeSize,
                 uint index [[thread_position_in_grid]])
{
    if (isInsideArr[index]) {
        return;
    }
    int3 position = positions[index];
    bool fixed = isFixedArr[index];
    bool wall = isWallArr[index];
    int f_start = index * ni;
    
    // ------------------------------------------------------------------------ //
    //                                                                          //
    // First figure out which is our read and write buffer using ij. We will    //
    // only read from currentF and only write to nextF. This avoids any race    //
    // conditions.                                                              //
    //                                                                          //
    // ------------------------------------------------------------------------ //
    
    float f[ni];
    if (fixed) {
        for (int i = 0; i < ni; i++) {
            f[i] = currentF[f_start + i];
        }
    } else {
        backStreaming(f, currentF, f_start, position, *latticeSize);
    }
    
    // Calculate the density and velocity
    float density = 0;
    float3 velocity = { 0, 0, 0 };
    for (int i = 0; i < ni; i++) {
        density += f[i];
        velocity += f[i] * float3(e[i]);
    }

    if (density > 0.00000005) {
        velocity /= density;
    } else {
        velocity = { 0, 0, 0 };
    }
    
    // Cap the velocity magnitude at the speed of sound
    float velocity_magnitude = length(velocity);
    if (velocity_magnitude > c) {
        velocity = velocity * c / velocity_magnitude;
    }
    
    // Update the density and velocity values
    densities[index] = density;
    velocities[index] = velocity;
    
    // We don't stream if the point is fixed
    if (fixed) {
        return;
    }
    
    // If we are at a wall, we apply the non-slip boundary condition
    if (wall) {
        for (int i = 0; i < ni; i++) {
            nextF[f_start + i] = f[opposite_pointers[i]];
        }
        return;
    }

    // ------------------------------------------------------------------------ //
    //                                                                          //
    // Run the collision pass. We calculate the equilibrium f for each velocity //
    // vector and 'relax' towards this value based on tau. Again we don't do    //
    // this for fixed lattice points.                                           //
    //                                                                          //
    // ------------------------------------------------------------------------ //
    
    if (not fixed) {
        float f_feq[ni];
        float u_sqrd = dot(velocity, velocity);
        for (int i = 0; i < ni; i++) {
            float eu = dot(float3(e[i]), velocity);
            float f_eq = w[i] * density * (1 + eu * inv_c_2 + 0.5 * eu * eu * inv_c_4 - 0.5 * u_sqrd * inv_c_2);
            f_feq[i] = f[i] - f_eq;
        }
        
        for (int i = 0; i < ni; i++) {
            float f_relax = 0;
            for (int j = 0; j < ni; j++) {
                // We are using MRT LBM
                f_relax += (StaticRelaxationMatrix[i][j] + s * KineticRelaxationMatrix[i][j]) * f_feq[j];
            }
            f[i] -= f_relax;
        }
    }
    
    for (int i = 0; i < ni; i++) {
        nextF[f_start + i] = f[i];
    }
}
