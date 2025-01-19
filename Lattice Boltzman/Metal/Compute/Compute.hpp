//
//  Compute.hpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 09/12/2024.
//

#ifndef Compute_hpp
#define Compute_hpp

#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>
#include <simd/simd.h>
#include <string>
#include <vector>
#include <AppKit/AppKit.hpp>
#include <MetalKit/MetalKit.hpp>


class Compute {
public:
    Compute(MTL::Device* device);
    void createInitialConditions(MTL::Device* device, MTL::CommandQueue* commandQueue, std::vector<simd_float3> &vertices, std::vector<int> &indices, simd_int3 minVertex, simd_int3 maxVertex, simd_int3 latticeSize);
    void sendComputeCommand(MTL::CommandBuffer* commandBuffer);
    void writeStateToFile(int frame);
    
    simd_int3 latticeSize;
    
    // Public buffers (as used by other shaders)
    MTL::Buffer* densityBuffer;
    MTL::Buffer* velocityBuffer;
    MTL::Buffer* latticeSizeBuffer;

private:
    void buildShaders(MTL::Device* device);
    void buildBuffers(MTL::Device* device, MTL::CommandQueue* commandQueue);
    float getEquilibrium(int i, float density, simd_float3 velocity);

    void setBuffers(MTL::ComputeCommandEncoder* computeEncoder);
    void encodeCommand(MTL::ComputeCommandEncoder* computeEncoder, MTL::ComputePipelineState* command);
            
    bool ij = false;
    int nLatticePoints;

    MTL::ComputePipelineState* _mStepPSO;

    MTL::Buffer* _fiBuffer;
    MTL::Buffer* _fjBuffer;
    MTL::Buffer* _positionsBuffer;
    MTL::Buffer* _fixedBuffer;
    MTL::Buffer* _wallBuffer;
    MTL::Buffer* _isInside;
};

#endif /* Compute_hpp */
