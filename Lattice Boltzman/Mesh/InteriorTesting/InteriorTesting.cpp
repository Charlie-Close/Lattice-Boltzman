//
//  InteriorTesting.cpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 09/12/2024.
//
//  MeshInsideTester class
//    -  Used to test whether a lattice point is inside the mesh.
//    -  Has been optimised, as we know every point we are going to test is an integer.
//       -  We build a grid where for each ray pointing in the x direction at an integer
//          yz we store all points where it intersects the mesh.
//       -  When we want to test a point, we can find the correspoinding ray in this grid
//          and count the number of intersections with a lower x value (could also use greater
//          it is equivalent).
//       -  If this value is odd, we are inside the mesh, otherwise we are outside the mesh.
//    -  The precomputing of this grid is necessary - without it start up can take multiple
//       minutes.
//

#include "InteriorTesting.hpp"
#include "Parameters.h"
#include "stl_reader.h"
#include <stdexcept>
#include <cmath>
#include <iostream>

// -------------------------------- //
//                                  //
//          Constructor             //
//                                  //
// -------------------------------- //

MeshInsideTester::MeshInsideTester(std::vector<simd_float3> &vertices, std::vector<int> &indices, simd_int3 minVertex, simd_int3 maxVertex):
_vertices(vertices), _indices(indices), _minVertex(minVertex), _maxVertex(maxVertex)
{
    buildGrid();
}

MeshInsideTester::~MeshInsideTester() {
    _intersections.clear();
}

// -------------------------------- //
//                                  //
//          Main function           //
//                                  //
// -------------------------------- //

bool MeshInsideTester::IsInside(int x, int y, int z) {
    int intersectionCount = 0;
    
    if (x < _minVertex.x or y < _minVertex.y or z < _minVertex.z or x >= _maxVertex.x or y >= _maxVertex.y or z >= _maxVertex.z) {
        return false;
    }
    
    simd_int3 gridArea = _maxVertex - _minVertex;
    int coord = (y - _minVertex.y) * gridArea.z + (z - _minVertex.z);
    std::vector<float> intersectionData = _intersections[coord];
    
    for (int i = 0; i < intersectionData.size(); i++) {
        if (intersectionData[i] < x) {
            intersectionCount++;
        }
    }

    return (intersectionCount % 2) == 1;
}

// -------------------------------- //
//                                  //
//        Helper function           //
//                                  //
// -------------------------------- //

bool MeshInsideTester::findIntersectionPoint(const simd_float3& orig, const simd_float3& dir,
                           const simd_float3& v0, const simd_float3& v1, const simd_float3& v2, simd_float3& intersectionPoint)
{
    const float EPSILON = 1e-9;
    simd_float3 edge1 = v1 - v0;
    simd_float3 edge2 = v2 - v0;
    simd_float3 h = simd::cross(dir, edge2);
    float a = simd::dot(edge1, h);

    if (std::fabs(a) < EPSILON) {
        // Ray is parallel to triangle
        return false;
    }

    float f = 1.0 / a;
    simd_float3 s = orig - v0;
    float u = f * simd::dot(s, h);
    if (u < 0.0 || u > 1.0) {
        return false;
    }

    simd_float3 q = simd::cross(s, edge1);
    float v = f * simd::dot(dir, q);
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }

    float t = f * simd::dot(edge2, q);
    if (t > EPSILON) {
        intersectionPoint = orig + dir * t;
        return true;
    }

    return false;
}

// -------------------------------- //
//                                  //
//           Precompute             //
//                                  //
// -------------------------------- //

void MeshInsideTester::buildGrid() {
    simd_int3 gridArea = _maxVertex - _minVertex;
    int gridSize = gridArea.y * gridArea.z;
    _intersections = std::vector<std::vector<float>>(gridSize);
    
    for (int i = 0; i < gridSize; i++) {
        _intersections[i] = std::vector<float>();
    }
    
    for (int i = 0; i < _indices.size(); i+=3) {
        simd_float3 v0 = _vertices[_indices[i]];
        simd_float3 v1 = _vertices[_indices[i+1]];
        simd_float3 v2 = _vertices[_indices[i+2]];
        
        // Compute bounding square:
        int minY = floor(fmin(v0.y, fmin(v1.y, v2.y)));
        int minZ = floor(fmin(v0.z, fmin(v1.z, v2.z)));
        int maxY = ceil(fmax(v0.y, fmax(v1.y, v2.y)));
        int maxZ = ceil(fmax(v0.z, fmax(v1.z, v2.z)));
        
        for (int y = minY; y < maxY; y++) {
            for (int z = minZ; z < maxZ; z++) {
                if (y < _minVertex.y or z < _minVertex.z or y >= _maxVertex.y or z >= _maxVertex.z) {
                    continue;
                }
                simd_float3 intersectionPoint;
                simd_float3 origin = { 0, (float)y, (float)z };
                bool intersected = findIntersectionPoint(origin, _rayDir, v0, v1, v2, intersectionPoint);
                if (intersected) {
                    int coord = (y - _minVertex.y) * gridArea.z + (z - _minVertex.z);
                    _intersections[coord].push_back(intersectionPoint.x);
                }
            }
        }
    }
}
