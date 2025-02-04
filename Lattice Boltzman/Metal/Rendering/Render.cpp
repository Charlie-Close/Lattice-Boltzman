//
//  Render.cpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 08/12/2024.
//

#include "Render.hpp"
#include "ViewAdapter.hpp"
#include "MetalHelpers.hpp"
#include "Parameters.h"
#include <iostream>

// -------------------------------- //
//                                  //
//          Constructor             //
//                                  //
// -------------------------------- //

Renderer::Renderer( MTL::Device* pDevice, Camera* camera )
: _pDevice( pDevice->retain() )
, _frame( 0 )
{
    _pCommandQueue = _pDevice->newCommandQueue();

    compute = new Compute(_pDevice);
    mesh = new Mesh(_pDevice, compute->velocityBuffer, compute->densityBuffer, compute->latticeSizeBuffer);
    compute->createInitialConditions(_pDevice, _pCommandQueue, mesh->vertices, mesh->indices, mesh->meshMinVertex, mesh->meshMaxVertex, mesh->simulationBoundingBox);
    particleTrails = new ParticleTrails(_pDevice, compute->densityBuffer, compute->velocityBuffer, compute->latticeSizeBuffer, compute->latticeSize);
    
    buildBuffers();
    
    this->camera = camera;
    _semaphore = dispatch_semaphore_create(1);
}

Renderer::~Renderer()
{
    _pCameraDataBuffer->release();
    _pCommandQueue->release();
    _pDevice->release();
}

void Renderer::buildBuffers()
{
    _pCameraDataBuffer = _pDevice->newBuffer( sizeof(simd::float4x4), MTL::ResourceStorageModeManaged );
}

// -------------------------------- //
//                                  //
//          Draw command            //
//                                  //
// -------------------------------- //

void Renderer::draw(MTK::View* pView)
{
    NS::AutoreleasePool* pPool = NS::AutoreleasePool::alloc()->init();
    MTL::CommandBuffer* pCmd = _pCommandQueue->commandBuffer();
    
    // Semaphore stuff
    dispatch_semaphore_wait(_semaphore, DISPATCH_TIME_FOREVER);
    Renderer* pRenderer = this;
    pCmd->addCompletedHandler(^void(MTL::CommandBuffer* pCmd) {
        dispatch_semaphore_signal(pRenderer->_semaphore);
    });
    
    for (int i = 0; i < stepsPerFrame; i++) {
        // Step the simulation
        compute->sendComputeCommand(pCmd);
        particleTrails->sendComputeCommand(pCmd);
        _frame++;
    }
    
    mesh->computeForces(pCmd);
    
    // Update camera matrix
    simd::float4x4* pCameraData = reinterpret_cast<simd::float4x4*>(_pCameraDataBuffer->contents());
    *pCameraData = camera->getMatrix();
    _pCameraDataBuffer->didModifyRange(NS::Range::Make(0, sizeof(simd::float4x4)));
    
    // Begin render pass
    MTL::RenderPassDescriptor* pRpd = pView->currentRenderPassDescriptor();
    MTL::RenderCommandEncoder* pEnc = pCmd->renderCommandEncoder(pRpd);

    mesh->draw(pEnc, _pCameraDataBuffer);
    particleTrails->draw(pEnc, _pCameraDataBuffer);

    pEnc->endEncoding();
    
    pCmd->presentDrawable(pView->currentDrawable());
    pCmd->commit();

    pPool->release();
}

