//
//  Mesh.hpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 18/01/2025.
//

#ifndef Mesh_hpp
#define Mesh_hpp

#include <Metal/Metal.hpp>
#include <vector>
#include <simd/simd.h>

class Mesh
{
public:
    Mesh(MTL::Device* device, MTL::Buffer* _velocityBuffer, MTL::Buffer* _densityBuffer, MTL::Buffer* _latticeSizeBuffer);
    
    void draw(MTL::RenderCommandEncoder *pEnc, MTL::Buffer* cameraDataBuffer);
    void computeForces(MTL::CommandBuffer* pCmd);
    
    simd_int3 simulationBoundingBox;
    simd_int3 meshMinVertex;
    simd_int3 meshMaxVertex;
    
    std::vector<simd_float3> vertices;
    std::vector<simd_float3> normals;
    std::vector<int> indices;

private:
    void loadMeshFromFile();
    void buildShaders(MTL::Device* device);
    void buildBuffers(MTL::Device* device);
    void buildDepthStencilStates(MTL::Device* device);
    void rotateMesh(float theta, float phi, float psi);
    void scaleMesh(float resolution);
    
    unsigned int nIndices;
    
    MTL::ComputePipelineState* _forcesPSO;
    MTL::RenderPipelineState* _renderPSO;
    
    // Depth stencil
    MTL::DepthStencilState* _depthStencilState;
    
    
    MTL::Buffer* _positionBuffer;
    MTL::Buffer* _normalBuffer;
    MTL::Buffer* _indexBuffer;
    MTL::Buffer* _forceBuffer;
    MTL::Buffer* _velocityBuffer;
    MTL::Buffer* _densityBuffer;
    MTL::Buffer* _latticeSizeBuffer;
};


#endif /* Mesh_hpp */
