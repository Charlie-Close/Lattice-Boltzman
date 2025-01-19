//
//  MetalHelpers.cpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 11/12/2024.
//

#include "MetalHelpers.hpp"

void copyDataToBuffer(MTL::CommandQueue* commandQueue, MTL::Buffer *privateBuffer, MTL::Buffer *sharedBuffer, unsigned long bufferLength) {
    MTL::CommandBuffer* commandBuffer = commandQueue->commandBuffer();
    
    MTL::BlitCommandEncoder* blitCommandEncoder = commandBuffer->blitCommandEncoder();
    
    blitCommandEncoder->copyFromBuffer(sharedBuffer, 0, privateBuffer, 0, bufferLength);
    blitCommandEncoder->endEncoding();
    
    commandBuffer->commit();
    commandBuffer->waitUntilCompleted();
}

void copyDataToTexture(MTL::CommandQueue* commandQueue, MTL::Texture *privateTexture, MTL::Texture *sharedTexture) {
    MTL::CommandBuffer* commandBuffer = commandQueue->commandBuffer();
    
    MTL::BlitCommandEncoder* blitCommandEncoder = commandBuffer->blitCommandEncoder();
    
    blitCommandEncoder->copyFromTexture(privateTexture, sharedTexture);
    blitCommandEncoder->endEncoding();
    
    commandBuffer->commit();
    commandBuffer->waitUntilCompleted();
}

void copyDataToTexture(MTL::CommandQueue* commandQueue, MTL::Texture *privateTexture, MTL::Buffer *sourceBuffer, unsigned long bufferLength, int width, int height, int depth) {
    MTL::CommandBuffer* commandBuffer = commandQueue->commandBuffer();
    
    MTL::BlitCommandEncoder* blitCommandEncoder = commandBuffer->blitCommandEncoder();
    
    int sourceBytesPerRow = width * sizeof(float) * 4;
    int sourceBytesPerImage = sourceBytesPerRow * height;
    MTL::Origin destinationOrigin(0, 0, 0);       // Start at the top-left of the 3D texture
    MTL::Size sourceSize(bufferLength, 1, 1);
    
    
    blitCommandEncoder->copyFromBuffer(sourceBuffer, 0, sourceBytesPerRow, sourceBytesPerImage, sourceSize, privateTexture, 0, 0, destinationOrigin, MTL::BlitOptionNone);
    blitCommandEncoder->endEncoding();
    
    commandBuffer->commit();
    commandBuffer->waitUntilCompleted();
}
