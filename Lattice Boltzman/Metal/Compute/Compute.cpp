//
//  Compute.cpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 09/12/2024.
//
//  Compute Class
//    -  Handles running the LBM simulation
//    -  Generates the initial conditions
//    -  Sends a compute command to simulate the fluid flow
//
//  Note - the use of two buffers to store f (the velocity distribution at each lattice point)
//  f_i and f_j. Each pass, on is used as readonly, and the other is write only. This avoids
//  any race conditions and a mismatch of what timestep each value represents.
//

#include "Compute.hpp"
#include "MetalHelpers.hpp"
#include "MeshHelpers.hpp"
#include "InteriorTesting.hpp"
#include "SimulationConstants.h"
#include "Parameters.h"
#include <iostream>
#include <fstream>


// -------------------------------- //
//                                  //
//   Initial Condition Functions    //
//                                  //
// -------------------------------- //

float initialDensity(int x, int y, int z, int max_x, int max_y, int max_z) {
    return 1.f;
}

simd_float3 initialVelocity(int x, int y, int z, int max_x, int max_y, int max_z) {
    if (x == 0) {
        return { airSpeed, 0, 0 };
    }
    return { 0, 0, 0 };
}
// Fixed points do not stream (i.e. have a constant velocity and density)
// We fix the x face to simulate a constant stream of fluid (like a wind tunnel)
bool isFixed(int x, int y, int z, int max_x, int max_y, int max_z) {
    if (x == 0) {
        return true;
    }
    return false;
}

// -------------------------------- //
//                                  //
//          Constructor             //
//                                  //
// -------------------------------- //

Compute::Compute(MTL::Device* device) {
    MTL::CommandQueue* commandQueue = device->newCommandQueue();

    buildShaders(device);
    buildBuffers(device, commandQueue);
}

// -------------------------------- //
//                                  //
//   Initial Condition Generation   //
//                                  //
// -------------------------------- //

float Compute::getEquilibrium(int i, float density, simd_float3 velocity) {
    float eu = e[i].x * velocity.x +
               e[i].y * velocity.y +
               e[i].z * velocity.z;
    float u_sqrd = velocity.x * velocity.x +
                    velocity.y * velocity.y +
                    velocity.z * velocity.z;
    return w[i] * density * (1 + eu * inv_c_2 + 0.5 * eu * eu * inv_c_4 - 0.5 * u_sqrd * inv_c_2);
}


void Compute::createInitialConditions(MTL::Device* device, MTL::CommandQueue* commandQueue, std::vector<simd_float3> &vertices, std::vector<int> &indices, simd_int3 minVertex, simd_int3 maxVertex, simd_int3 latticeSize) {
    // Create the "MeshInsideTester" - a class which can efficiently tell us whether a lattice point is inside the mesh.
    MeshInsideTester* tester = new MeshInsideTester(vertices, indices, minVertex, maxVertex);
    
    this->latticeSize = latticeSize;
    nLatticePoints = latticeSize[0] * latticeSize[1] * latticeSize[2];
    
    // Create the arrays to store the initial conditions
    uint fSize = nLatticePoints * ni;
    float* f = new float[fSize];
    simd_int3* positions = new simd_int3[nLatticePoints];
    bool* fixed = new bool[nLatticePoints];
    bool* wall = new bool[nLatticePoints];
    bool* inside = new bool[nLatticePoints];
    
    // For every lattice point
    for (int z = 0; z < latticeSize[2]; z++) {
        for (int y = 0; y < latticeSize[1]; y++) {
            for (int x = 0; x < latticeSize[0]; x++) {
                // Get values at the lattice point
                float localDensity = initialDensity(x, y, z, latticeSize[0], latticeSize[1], latticeSize[2]);
                simd_float3 localVelocity = initialVelocity(x, y, z, latticeSize[0], latticeSize[1], latticeSize[2]);
                bool localFixed = isFixed(x, y, z, latticeSize[0], latticeSize[1], latticeSize[2]);
                bool localWall = tester->IsInside(x, y, z);
                
                // Set values in their correspending arrays
                int latticeNumber = x + y * latticeSize[0] + z * latticeSize[0] * latticeSize[1];
                positions[latticeNumber] = { x, y, z };
                fixed[latticeNumber] = localFixed;
                wall[latticeNumber] = localWall;
                inside[latticeNumber] = false;
                
                for (int i = 0; i < ni; i++) {
                    // Initial conditions is the equilibrium velocity distribution for a given density and velocity
                    f[latticeNumber * ni + i] = getEquilibrium(i, localDensity, localVelocity);
                }
            }
        }
    }
    
    delete tester;
    
    // Write all the data to the buffers
    writeDataToPrivateBuffer(device, commandQueue, _fiBuffer, f, fSize);
    writeDataToPrivateBuffer(device, commandQueue, _fjBuffer, f, fSize);
    writeDataToPrivateBuffer(device, commandQueue, _positionsBuffer, positions, nLatticePoints);
    writeDataToPrivateBuffer(device, commandQueue, _fixedBuffer, fixed, nLatticePoints);
    writeDataToPrivateBuffer(device, commandQueue, _wallBuffer, wall, nLatticePoints);
    writeDataToPrivateBuffer(device, commandQueue, _isInside, inside, nLatticePoints);
    writeDataToPrivateBuffer(device, commandQueue, latticeSizeBuffer, latticeSize);
}

// -------------------------------- //
//                                  //
//      Metal Object Builders       //
//                                  //
// -------------------------------- //

void Compute::buildBuffers(MTL::Device* device, MTL::CommandQueue *commandQueue) {
    _fiBuffer = device->newBuffer(resolution * ni * sizeof(float), MTL::ResourceStorageModePrivate);
    _fjBuffer = device->newBuffer(resolution * ni * sizeof(float), MTL::ResourceStorageModePrivate);
    densityBuffer = device->newBuffer(resolution * sizeof(float), MTL::ResourceStorageModePrivate);
    _positionsBuffer = device->newBuffer(resolution * sizeof(simd_int3), MTL::ResourceStorageModePrivate);
    velocityBuffer = device->newBuffer(resolution * sizeof(simd_float3), MTL::ResourceStorageModePrivate);
    _fixedBuffer = device->newBuffer(resolution * sizeof(bool), MTL::ResourceStorageModePrivate);
    _wallBuffer = device->newBuffer(resolution * sizeof(bool), MTL::ResourceStorageModePrivate);
    _isInside = device->newBuffer(resolution * sizeof(bool), MTL::ResourceStorageModePrivate);
    latticeSizeBuffer = device->newBuffer(sizeof(simd_int3), MTL::ResourceStorageModePrivate);
}

void Compute::buildShaders(MTL::Device* device) {
    NS::Error** error = nil;

    // Load the shader files with a .metal file extension in the project
    MTL::Library* defaultLibrary = device->newDefaultLibrary();
        
    NS::String* stepFunctionName = NS::String::string("step", NS::StringEncoding::UTF8StringEncoding);
    MTL::Function* stepFunction = defaultLibrary->newFunction(stepFunctionName);
    
    // Create a compute pipeline state object.
    _mStepPSO = device->newComputePipelineState(stepFunction, error);
}

// -------------------------------- //
//                                  //
//          Compute Command         //
//                                  //
// -------------------------------- //

void Compute::sendComputeCommand(MTL::CommandBuffer* commandBuffer) {
    // Start a compute pass.
    MTL::ComputeCommandEncoder* computeEncoder = commandBuffer->computeCommandEncoder();
    assert(computeEncoder != nil);
    
    ij = not ij;
    setBuffers(computeEncoder);
    encodeCommand(computeEncoder, _mStepPSO);

    //  End the compute pass.
    computeEncoder->endEncoding();
}

void Compute::setBuffers(MTL::ComputeCommandEncoder* computeEncoder) {
    if (ij) {
        computeEncoder->setBuffer(_fiBuffer, 0, 0);
        computeEncoder->setBuffer(_fjBuffer, 0, 1);
    } else {
        computeEncoder->setBuffer(_fjBuffer, 0, 0);
        computeEncoder->setBuffer(_fiBuffer, 0, 1);
    }
    computeEncoder->setBuffer(densityBuffer, 0, 2);
    computeEncoder->setBuffer(_positionsBuffer, 0, 3);
    computeEncoder->setBuffer(velocityBuffer, 0, 4);
    computeEncoder->setBuffer(_fixedBuffer, 0, 5);
    computeEncoder->setBuffer(_wallBuffer, 0, 6);
    computeEncoder->setBuffer(_isInside, 0, 7);
    computeEncoder->setBuffer(latticeSizeBuffer, 0, 8);
}

void Compute::encodeCommand(MTL::ComputeCommandEncoder* computeEncoder, MTL::ComputePipelineState* command) {
    MTL::Size gridSize = MTL::Size(nLatticePoints, 1, 1);

    computeEncoder->setComputePipelineState(command);
    // Calculate a threadgroup size.
    NS::UInteger threadGroupSize = command->maxTotalThreadsPerThreadgroup();
    if (threadGroupSize > nLatticePoints)
    {
        threadGroupSize = nLatticePoints;
    }
    MTL::Size threadgroupSize = MTL::Size(512, 1, 1);

    // Encode the compute command.
    computeEncoder->dispatchThreads(gridSize, threadgroupSize);
}

// -------------------------------- //
//                                  //
//          Data Generation         //
//                                  //
// -------------------------------- //

void Compute::writeStateToFile(int frame) {
    std::string filename = "Results/data" + std::to_string(frame) + ".txt";

    // Open the file in binary mode
    std::ofstream MyFile(filename, std::ios::binary);
    if (!MyFile) {
        // Handle error opening file if necessary
        return;
    }

    int firstInt = latticeSize.x;
    int secondInt = latticeSize.y;
    int thirdInt = latticeSize.z;
    MyFile.write(reinterpret_cast<char*>(&firstInt),   sizeof(firstInt));
    MyFile.write(reinterpret_cast<char*>(&secondInt),  sizeof(secondInt));
    MyFile.write(reinterpret_cast<char*>(&thirdInt),   sizeof(thirdInt));

    // Cast buffers
    void* voidVelBuffer  = velocityBuffer->contents();
    void* voidDensBuffer = densityBuffer->contents();
    void* voidWallBuffer = _wallBuffer->contents();
    simd_float3* velocityBuffer = static_cast<simd_float3*>(voidVelBuffer);
    float* densityBuffer = static_cast<float*>(voidDensBuffer);
    bool* wallBuffer = static_cast<bool*>(voidWallBuffer);
    
    
    // Write out all points in binary
    for (int i = 0; i < nLatticePoints; i++) {
        // Write velocity x, y, z
        float px = velocityBuffer[i].x;
        float py = velocityBuffer[i].y;
        float pz = velocityBuffer[i].z;
        MyFile.write(reinterpret_cast<char*>(&px), sizeof(float));
        MyFile.write(reinterpret_cast<char*>(&py), sizeof(float));
        MyFile.write(reinterpret_cast<char*>(&pz), sizeof(float));

        // Write density
        MyFile.write(reinterpret_cast<char*>(&densityBuffer[i]), sizeof(float));
        
        // Write wall data
        MyFile.write(reinterpret_cast<char*>(&wallBuffer[i]), sizeof(bool));
    }
    
    MyFile.close();
}
