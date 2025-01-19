//
//  MetalHelpers.hpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 09/12/2024.
//

#ifndef MetalHelpers_hpp
#define MetalHelpers_hpp

#include <stdio.h>
#include <vector>
#include <Metal/Metal.hpp>

void copyDataToBuffer(MTL::CommandQueue* commandQueue, MTL::Buffer *privateBuffer, MTL::Buffer *sharedBuffer, unsigned long bufferLength);

void copyDataToTexture(MTL::CommandQueue* commandQueue, MTL::Texture *privateTexture, MTL::Texture *sharedTexture);


template <typename T>
void writeDataToBuffer(MTL::Buffer *buffer, std::vector<T> data) {
    void* voidBuffer = buffer->contents();
    T* castBuffer = static_cast<T*>(voidBuffer);
    
    for (int i = 0; i < data.size(); i++) {
        castBuffer[i] = data[i];
    }
}

template <typename T>
void writeDataToBuffer(MTL::Buffer *buffer, T *data, unsigned long n) {
    void* voidBuffer = buffer->contents();
    T* castBuffer = static_cast<T*>(voidBuffer);
    
    for (int i = 0; i < n; i++) {
        castBuffer[i] = data[i];
    }
}

template <typename T>
void writeDataToBuffer(MTL::Buffer *buffer, T data) {
    // Get the buffer contents
    void* voidBuffer = buffer->contents();
    
    // Cast the contents to a float pointer and assign the value
    T* castBuffer = static_cast<T*>(voidBuffer);
    *castBuffer = data;
}

template <typename T>
void writeDataToPrivateBuffer(MTL::Device* device, MTL::CommandQueue* commandQueue, MTL::Buffer *privateBuffer, std::vector<T> data) {
    MTL::Buffer* sharedBuffer = device->newBuffer(data.size() * sizeof(T), MTL::ResourceStorageModeShared);
    writeDataToBuffer(sharedBuffer, data);
    copyDataToBuffer(commandQueue, privateBuffer, sharedBuffer, data.size() * sizeof(T));
    sharedBuffer->release();
}

template <typename T>
void writeDataToPrivateBuffer(MTL::Device* device, MTL::CommandQueue* commandQueue, MTL::Buffer *privateBuffer, T *data, unsigned long n) {
    MTL::Buffer* sharedBuffer = device->newBuffer(n * sizeof(T), MTL::ResourceStorageModeShared);
    writeDataToBuffer(sharedBuffer, data, n);
    copyDataToBuffer(commandQueue, privateBuffer, sharedBuffer, n * sizeof(T));
    sharedBuffer->release();
}

template <typename T>
void writeDataToPrivateBuffer(MTL::Device* device, MTL::CommandQueue* commandQueue, MTL::Buffer *privateBuffer, T data) {
    MTL::Buffer* sharedBuffer = device->newBuffer(sizeof(T), MTL::ResourceStorageModeShared);
    writeDataToBuffer(sharedBuffer, data);
    copyDataToBuffer(commandQueue, privateBuffer, sharedBuffer, sizeof(T));
    sharedBuffer->release();
}

template <typename T>
void writeDataToPrivateTexture(MTL::Device* device, MTL::CommandQueue* commandQueue, MTL::Buffer *privateBuffer, T data) {
    MTL::Buffer* sharedBuffer = device->newBuffer(sizeof(T), MTL::ResourceStorageModeShared);
    writeDataToBuffer(sharedBuffer, data);
    copyDataToBuffer(commandQueue, privateBuffer, sharedBuffer, sizeof(T));
    sharedBuffer->release();
}

void copyDataToTexture(MTL::CommandQueue* commandQueue, MTL::Texture *privateTexture, MTL::Buffer *sourceBuffer, unsigned long bufferLength, int width, int height, int depth);


#endif /* MetalHelpers_hpp */
