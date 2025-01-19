//
//  MeshHelpers.cpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 18/01/2025.
//

#include "MeshHelpers.hpp"
#include <map>


// Function to mark all points surrounded by walls. This is an optimisation - we don't need
// to simulate points within the mesh.
void markInsidePoints(bool* grid, simd_int3 latticeSize, bool* isInside) {
    int xSize = latticeSize.x, ySize = latticeSize.y, zSize = latticeSize.z;
    // Function to calculate the 1D array index from a 3D position
    auto getIndex = [xSize, ySize, zSize](int x, int y, int z) -> int {
        return x + y * xSize + z * xSize * ySize;
    };
    // Mark interior points
    for (int x = 1; x < xSize - 1; ++x) {
        for (int y = 1; y < ySize - 1; ++y) {
            for (int z = 1; z < zSize - 1; ++z) {
                if (!grid[getIndex(x, y, z)]) {
                    // Check neighbors
                    bool isSurrounded = true;
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dz = -1; dz <= 1; ++dz) {
                                if (!grid[getIndex(x + dx, y + dy, z + dz)]) {
                                    isSurrounded = false;
                                    break;
                                }
                            }
                            if (!isSurrounded) break;
                        }
                        if (!isSurrounded) break;
                    }
                    
                    if (isSurrounded) {
                        isInside[getIndex(x, y, z)] = true;
                    }
                }
            }
        }
    }
}

std::tuple<float, float, float> makeVertexKey(const simd_float3& v) {
    return std::make_tuple(v.x, v.y, v.z);
}

// Removes repeated vertices and averages their normals (provides nicer shading)
void reduceAndAddNormals(std::vector<simd_float3> &vertices, std::vector<simd_float3> &normals, std::vector<int> &indices) {
    
    std::vector<simd_float3> uniqueVerts;
    std::vector<int> newFaces;
    std::map<std::tuple<float, float, float>, unsigned int> vertexMap;
    
    for (unsigned int i = 0; i < indices.size(); ++i) {
        simd_float3 vertex = vertices[indices[i]];
        auto vertexKey = makeVertexKey(vertex);

        // Check if vertex already exists in map
        if (vertexMap.find(vertexKey) == vertexMap.end()) {
            // Add new unique vertex
            uniqueVerts.push_back(vertex);
            unsigned int newIndex = uniqueVerts.size() - 1;
            vertexMap[vertexKey] = newIndex;
        }
        // Use the index of the unique vertex
        newFaces.push_back(vertexMap[vertexKey]);
    }
    
    vertices = uniqueVerts;
    indices = newFaces;
    
    // Calculate normals
    normals = std::vector<simd_float3>(vertices.size(), (simd_float3){ 0.f, 0.f, 0.f }); // Initialize normals with 0s

    for (unsigned int i = 0; i < indices.size(); i += 3) {
        // Indices of the triangle vertices
        unsigned int i0 = indices[i];
        unsigned int i1 = indices[i + 1];
        unsigned int i2 = indices[i + 2];

        // Vertex positions
        simd_float3 v0 = vertices[i0];
        simd_float3 v1 = vertices[i1];
        simd_float3 v2 = vertices[i2];

        // Compute face normal
        simd_float3 edge1 = v1 - v0;
        simd_float3 edge2 = v2 - v0;
        simd_float3 faceNormal = simd::normalize(simd::cross(edge1, edge2));

        // Accumulate face normal to the vertices
        for (unsigned int vertexIndex : {i0, i1, i2}) {
            normals[vertexIndex] += faceNormal;
        }
    }

    // Normalize the normals
    for (size_t i = 0; i < normals.size(); i++) {
        normals[i] = simd::normalize(normals[i]);
    }
}

