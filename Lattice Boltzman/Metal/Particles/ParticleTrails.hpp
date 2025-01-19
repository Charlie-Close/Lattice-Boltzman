//
//  ParticleTrails.hpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 09/12/2024.
//

#ifndef ParticleTrails_hpp
#define ParticleTrails_hpp

#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>
#include <simd/simd.h>
#include <string>
#include <vector>
#include <AppKit/AppKit.hpp>
#include <MetalKit/MetalKit.hpp>

class ParticleTrails {
public:
    ParticleTrails(MTL::Device* _device, MTL::Buffer* _densityBuffer, MTL::Buffer* _velocityBuffer, MTL::Buffer* _latticeSizeBuffer, simd_int3 latticeSize);
    void sendComputeCommand(MTL::CommandBuffer* commandBuffer);
    void draw(MTL::RenderCommandEncoder *pEnc, MTL::Buffer* cameraDataBuffer);


private:
    void buildShaders(MTL::Device* device);
    void buildDepthStencilStates(MTL::Device* device);
    void buildBuffers(MTL::Device* device, MTL::CommandQueue* commandQueue);
    void setInitialConditions(MTL::Device* device, MTL::CommandQueue* commandQueue, simd_int3 latticeSize);

    void incrementLoopCommand(MTL::CommandBuffer* commandBuffer);
    void setBuffers(MTL::ComputeCommandEncoder* computeEncoder);
    void encodeCommand(MTL::ComputeCommandEncoder* computeEncoder, MTL::ComputePipelineState* command);
    
    int loopPos = 0;
    int nPoints;

    // Pipleline State Objects
    MTL::ComputePipelineState* _incrementLoopPSO;
    MTL::ComputePipelineState* _mParticlePSO;
    MTL::RenderPipelineState* _drawPSO;
    
    // Depth stencil
    MTL::DepthStencilState* _depthStencilState;
    
    // Buffers
    MTL::Buffer* _positionBuffer;
    MTL::Buffer* _densityBuffer;
    MTL::Buffer* _velocityBuffer;
    MTL::Buffer* _loopPosBuffer;
    
    MTL::Buffer* _latticeDensityBuffer;
    MTL::Buffer* _latticeVelocityBuffer;
    MTL::Buffer* _latticeSizeBuffer;
};

#endif /* ParticleTrails_hpp */
