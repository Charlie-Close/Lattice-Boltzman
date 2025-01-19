//
//  ParticleTrails.cpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 09/12/2024.
//
//  Particle trails class:
//    -  Randomly generates a number of particles with certain trail lengths
//    -  Computes how these particles will move based on the fluid state
//    -  Colours particles based on density
//    -  Renders particles
//

#include "ParticleTrails.hpp"
#include "MetalHelpers.hpp"
#include "Parameters.h"

// Helper function to generate a random position within the lattice
simd_float3 getRandomPos(simd_int3 latticeSize) {
    simd_float3 pos;
    pos.x = (float)arc4random_uniform((uint32_t)latticeSize.x * 100) / 100;
    pos.y = (float)arc4random_uniform((uint32_t)latticeSize.y * 100) / 100;
    pos.z = (float)arc4random_uniform((uint32_t)latticeSize.z * 100) / 100;
    
    if (narrow) {
        pos.z = pos.z * 0.001 + latticeSize.z * 0.4995;
    }
    
    return pos;
}

// -------------------------------- //
//                                  //
//          Constructor             //
//                                  //
// -------------------------------- //

ParticleTrails::ParticleTrails(MTL::Device* device, MTL::Buffer* densityBuffer, MTL::Buffer* velocityBuffer, MTL::Buffer* latticeSizeBuffer, simd_int3 latticeSize):
_latticeDensityBuffer(densityBuffer), _latticeVelocityBuffer(velocityBuffer), _latticeSizeBuffer(latticeSizeBuffer)
{
    nPoints = nParticles * streakLength;

    buildShaders(device);
    buildDepthStencilStates(device);
    
    MTL::CommandQueue* commandQueue = device->newCommandQueue();
    buildBuffers(device, commandQueue);
    setInitialConditions(device, commandQueue, latticeSize);
}

// -------------------------------- //
//                                  //
//      Initial Conditions          //
//                                  //
// -------------------------------- //

void ParticleTrails::setInitialConditions(MTL::Device *device, MTL::CommandQueue *commandQueue, simd_int3 latticeSize) {
    simd_float3* initialPositions = new simd_float3[nPoints];
    for (int i = 0; i < nParticles; i++) {
        simd_float3 pos = getRandomPos(latticeSize);
        for (int j = 0; j < streakLength; j++) {
            initialPositions[i * streakLength + j] = pos;
        }
    }
    
    writeDataToPrivateBuffer(device, commandQueue, _positionBuffer, initialPositions, nPoints);
    writeDataToBuffer(_loopPosBuffer, loopPos);
}

// -------------------------------- //
//                                  //
//      Metal Object Builders       //
//                                  //
// -------------------------------- //

void ParticleTrails::buildBuffers(MTL::Device* device, MTL::CommandQueue *commandQueue) {
    _positionBuffer = device->newBuffer(nPoints * sizeof(simd_float3), MTL::ResourceStorageModePrivate);
    _densityBuffer = device->newBuffer(nPoints * sizeof(float), MTL::ResourceStorageModePrivate);
    _velocityBuffer = device->newBuffer(nPoints * sizeof(simd_float3), MTL::ResourceStorageModePrivate);
    _loopPosBuffer = device->newBuffer(sizeof(int), MTL::ResourceStorageModeShared);
}

void ParticleTrails::buildShaders(MTL::Device* device) {
    NS::Error** error = nil;

    // Load the shader files with a .metal file extension in the project
    MTL::Library* defaultLibrary = device->newDefaultLibrary();
    
    MTL::Function* particleFunction = defaultLibrary->newFunction(NS::String::string("particles", NS::StringEncoding::UTF8StringEncoding));
    MTL::Function* loopFunction = defaultLibrary->newFunction( NS::String::string("incrementLoopPos", NS::StringEncoding::UTF8StringEncoding) );
    MTL::Function* pVertexPn = defaultLibrary->newFunction( NS::String::string("vertexPoint", NS::StringEncoding::UTF8StringEncoding) );
    MTL::Function* pFragPn = defaultLibrary->newFunction( NS::String::string("fragmentPoint", NS::StringEncoding::UTF8StringEncoding) );
    
    // Create a compute pipeline state object.
    _mParticlePSO = device->newComputePipelineState(particleFunction, error);
    _incrementLoopPSO = device->newComputePipelineState(loopFunction, error);
    
    MTL::RenderPipelineDescriptor* pDesc = MTL::RenderPipelineDescriptor::alloc()->init();
    pDesc->setVertexFunction( pVertexPn );
    pDesc->setFragmentFunction( pFragPn );
    pDesc->colorAttachments()->object(0)->setPixelFormat( MTL::PixelFormat::PixelFormatBGRA8Unorm_sRGB );
    pDesc->setDepthAttachmentPixelFormat( MTL::PixelFormat::PixelFormatDepth16Unorm );
    _drawPSO = device->newRenderPipelineState( pDesc, error );

    pVertexPn->release();
    pFragPn->release();
    pDesc->release();
}

void ParticleTrails::buildDepthStencilStates(MTL::Device* device) {
    MTL::DepthStencilDescriptor* pDsDesc = MTL::DepthStencilDescriptor::alloc()->init();
    pDsDesc->setDepthCompareFunction( MTL::CompareFunction::CompareFunctionLess );
    // Attempting to write one million values to the depth stencil is a poor idea.
    // Therefore just in Renderer.cpp we need to make this draw last.
    pDsDesc->setDepthWriteEnabled( false );
    _depthStencilState = device->newDepthStencilState( pDsDesc );
    pDsDesc->release();
}

// -------------------------------- //
//                                  //
//          Compute Command         //
//                                  //
// -------------------------------- //

void ParticleTrails::sendComputeCommand(MTL::CommandBuffer* commandBuffer) {
    // First increment position in the loop
    incrementLoopCommand(commandBuffer);
    
    // Then run the compute command
    MTL::ComputeCommandEncoder* computeEncoder = commandBuffer->computeCommandEncoder();
    assert(computeEncoder != nil);

    setBuffers(computeEncoder);
    encodeCommand(computeEncoder, _mParticlePSO);

    computeEncoder->endEncoding();
}

void ParticleTrails::incrementLoopCommand(MTL::CommandBuffer* commandBuffer) {
    MTL::ComputeCommandEncoder* computeEncoder = commandBuffer->computeCommandEncoder();
    assert(computeEncoder != nil);
    
    computeEncoder->setBuffer(_loopPosBuffer, 0, 0);
    
    MTL::Size gridSize = MTL::Size(1, 1, 1);
    computeEncoder->setComputePipelineState(_incrementLoopPSO);
    MTL::Size threadgroupSize = MTL::Size(1, 1, 1);
    computeEncoder->dispatchThreads(gridSize, threadgroupSize);

    computeEncoder->endEncoding();
}

void ParticleTrails::setBuffers(MTL::ComputeCommandEncoder* computeEncoder) {
    computeEncoder->setBuffer(_positionBuffer, 0, 0);
    computeEncoder->setBuffer(_densityBuffer, 0, 1);
    computeEncoder->setBuffer(_velocityBuffer, 0, 2);
    computeEncoder->setBuffer(_latticeDensityBuffer, 0, 3);
    computeEncoder->setBuffer(_latticeVelocityBuffer, 0, 4);
    computeEncoder->setBuffer(_latticeSizeBuffer, 0, 5);
    computeEncoder->setBuffer(_loopPosBuffer, 0, 6);
}

void ParticleTrails::encodeCommand(MTL::ComputeCommandEncoder* computeEncoder, MTL::ComputePipelineState* command) {
    MTL::Size gridSize = MTL::Size(nParticles, 1, 1);

    computeEncoder->setComputePipelineState(command);
    // Calculate a threadgroup size.
    NS::UInteger threadGroupSize = _mParticlePSO->maxTotalThreadsPerThreadgroup();
    if (threadGroupSize > nParticles)
    {
        threadGroupSize = nParticles;
    }
    MTL::Size threadgroupSize = MTL::Size(threadGroupSize, 1, 1);

    // Encode the compute command.
    computeEncoder->dispatchThreads(gridSize, threadgroupSize);
}

// -------------------------------- //
//                                  //
//              Drawing             //
//                                  //
// -------------------------------- //

void ParticleTrails::draw(MTL::RenderCommandEncoder *pEnc, MTL::Buffer* cameraDataBuffer) {
    pEnc->setRenderPipelineState(_drawPSO);
    pEnc->setDepthStencilState(_depthStencilState);

    // Now draw points:
    pEnc->setVertexBuffer(_positionBuffer, 0, 0);
    pEnc->setVertexBuffer(_densityBuffer, 0, 1);
    pEnc->setVertexBuffer(_velocityBuffer, 0, 2);
    pEnc->setVertexBuffer(cameraDataBuffer, 0, 3);
    pEnc->drawPrimitives(MTL::PrimitiveTypePoint, (NS::UInteger)0, nPoints);
}
