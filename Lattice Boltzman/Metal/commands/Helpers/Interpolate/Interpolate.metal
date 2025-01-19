//
//  Interpolate.metal
//  Lattice Boltzman
//
//  Created by Charlie Close on 18/12/2024.
//

#include "Interpolate.h"

float4 interpolate(float3 position, int3 latticeSize, device const float3* velocities, device const float* densities) {
    int x0 = static_cast<int>(floor(position.x));
    int y0 = static_cast<int>(floor(position.y));
    int z0 = static_cast<int>(floor(position.z));
    
    // Ensure indices are within bounds
    x0 = max(0, min(x0, latticeSize.x - 1));
    y0 = max(0, min(y0, latticeSize.y - 1));
    z0 = max(0, min(z0, latticeSize.z - 1));
    
    float wx0 = position.x - (float)x0;
    float wy0 = position.y - (float)y0;
    float wz0 = position.z - (float)z0;

        
    float4 velocityAndDensity = { 0, 0, 0, 0 };
    
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
                int px = max(0, min(x0 + x, latticeSize.x - 1));
                int py = max(0, min(y0 + y, latticeSize.y - 1));
                int pz = max(0, min(z0 + z, latticeSize.z - 1));
                
                float wx = wx0 * x + (1.f - wx0) * (1.f - x);
                float wy = wy0 * y + (1.f - wy0) * (1.f - y);
                float wz = wz0 * z + (1.f - wz0) * (1.f - z);
                float w = wx * wy * wz;
                
                int index = px + py * latticeSize.x + pz * latticeSize.x * latticeSize.y;
                
                float3 vel = velocities[index];
                float dens = densities[index];
                velocityAndDensity += w * float4(vel, dens);
            }
        }
    }
    
    return velocityAndDensity;
    
}
