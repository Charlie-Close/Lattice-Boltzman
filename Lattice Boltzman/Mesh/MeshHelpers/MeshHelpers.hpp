//
//  MeshHelpers.hpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 18/01/2025.
//

#ifndef MeshHelpers_hpp
#define MeshHelpers_hpp

#include <stdio.h>
#include <simd/simd.h>
#include <vector>

void reduceAndAddNormals(std::vector<simd_float3> &vertices, std::vector<simd_float3> &normals, std::vector<int> &indices);

void markInsidePoints(bool* grid, simd_int3 latticeSize, bool* isInside);

#endif /* MeshHelpers_hpp */
