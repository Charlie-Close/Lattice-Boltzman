//
//  InteriorTesting.hpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 09/12/2024.
//

#ifndef InteriorTesting_hpp
#define InteriorTesting_hpp

#include <vector>
#include <simd/simd.h>

class MeshInsideTester {
public:
    MeshInsideTester(std::vector<simd_float3> &vertices, std::vector<int> &indices, simd_int3 minVertex, simd_int3 maxVertex);
    ~MeshInsideTester();

    bool IsInside(int x, int y, int z);
    void buildGrid();

private:
    std::vector<simd_float3> _vertices;
    std::vector<int> _indices;
    std::vector<std::vector<float>> _intersections;
    
    simd_int3 _minVertex;
    simd_int3 _maxVertex;
    
    simd_float3 _rayDir = { 1.0, 0.0, 0.0 };
    
    bool findIntersectionPoint(const simd_float3& orig, const simd_float3& dir,
                               const simd_float3& v0, const simd_float3& v1, const simd_float3& v2, simd_float3& intersectionPoint);
};

#endif /* InteriorTesting_hpp */
