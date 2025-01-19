//
//  Mesh.cpp
//  Lattice Boltzman
//
//  Created by Charlie Close on 18/01/2025.
//
//  Mesh class:
//    -  Reads in the stl file set in Parameters.h
//    -  Sets a bounding box shape based on the mesh and the resolution
//    -  Scales mesh to fit inside bounding box and rotates based on Parameters.h
//    -  Computes forces acting on each vertex of the mesh
//    -  Renders the mesh with colour based on the forces
//

#include "Mesh.hpp"
#include "stl_reader.h"
#include "Parameters.h"
#include "MetalHelpers.hpp"
#include "MeshHelpers.hpp"
#include <iostream>

// -------------------------------- //
//                                  //
//            Constructor           //
//                                  //
// -------------------------------- //

Mesh::Mesh(MTL::Device* device, MTL::Buffer* velocityBuffer, MTL::Buffer* densityBuffer, MTL::Buffer* latticeSizeBuffer):
_velocityBuffer(velocityBuffer), _densityBuffer(densityBuffer), _latticeSizeBuffer(latticeSizeBuffer)
{
    loadMeshFromFile();
    rotateMesh(theta, phi, psi);
    scaleMesh(resolution);
    reduceAndAddNormals(vertices, normals, indices);
    
    buildShaders(device);
    buildBuffers(device);
    buildDepthStencilStates(device);
    
    nIndices = indices.size();
}

void Mesh::loadMeshFromFile() {
    std::vector<float> _tmpVertices;
    std::vector<float> _tmpNormals;
    std::vector<int> _solidRanges;
    if(!stl_reader::ReadStlFile(filename, _tmpVertices, _tmpNormals, indices, _solidRanges)) {
        throw std::runtime_error("Failed to read STL file.");
    }
    for (int i = 0; i < _tmpVertices.size(); i += 3) {
        vertices.push_back((simd_float3){ _tmpVertices[i], _tmpVertices[i + 1], _tmpVertices[i + 2] });
    }
    
    _tmpVertices.clear();
    _tmpNormals.clear();
    _solidRanges.clear();
}

// -------------------------------- //
//                                  //
//      Metal Object Builders       //
//                                  //
// -------------------------------- //

void Mesh::buildShaders(MTL::Device* device)
{
    MTL::Library* pLibrary = device->newDefaultLibrary();
    NS::Error* pError = nullptr;
    
    if ( !pLibrary )
    {
        __builtin_printf( "%s", pError->localizedDescription()->utf8String() );
        assert( false );
    }
    
    MTL::Function* forcesFunction = pLibrary->newFunction( NS::String::string("forceCalculation", NS::StringEncoding::UTF8StringEncoding) );
    _forcesPSO = device->newComputePipelineState(forcesFunction, &pError);
    
    MTL::Function* pVertexFn = pLibrary->newFunction( NS::String::string("vertexMain", NS::StringEncoding::UTF8StringEncoding) );
    MTL::Function* pFragFn = pLibrary->newFunction( NS::String::string("fragmentMain", NS::StringEncoding::UTF8StringEncoding) );

    MTL::RenderPipelineDescriptor* pDesc = MTL::RenderPipelineDescriptor::alloc()->init();
    pDesc->setVertexFunction( pVertexFn );
    pDesc->setFragmentFunction( pFragFn );
    pDesc->colorAttachments()->object(0)->setPixelFormat( MTL::PixelFormat::PixelFormatBGRA8Unorm_sRGB );
    pDesc->setDepthAttachmentPixelFormat( MTL::PixelFormat::PixelFormatDepth16Unorm );

    _renderPSO = device->newRenderPipelineState( pDesc, &pError );
    if ( !_renderPSO )
    {
        __builtin_printf( "%s", pError->localizedDescription()->utf8String() );
        assert( false );
    }

    pVertexFn->release();
    pFragFn->release();
    pDesc->release();
    pLibrary->release();
}

void Mesh::buildBuffers(MTL::Device* device) {
    _forceBuffer = device->newBuffer(vertices.size() * sizeof(float), MTL::ResourceStorageModeShared);
    _positionBuffer = device->newBuffer(vertices.size() * sizeof(simd_float3), MTL::ResourceStorageModeShared);
    _normalBuffer = device->newBuffer(normals.size() * sizeof(simd_float3), MTL::ResourceStorageModeShared);
    _indexBuffer = device->newBuffer(indices.size() * sizeof(int), MTL::ResourceStorageModeShared);
    
    writeDataToBuffer(_positionBuffer, vertices);
    writeDataToBuffer(_normalBuffer, normals);
    writeDataToBuffer(_indexBuffer, indices);
}

void Mesh::buildDepthStencilStates(MTL::Device* device) {
    MTL::DepthStencilDescriptor* pDsDesc = MTL::DepthStencilDescriptor::alloc()->init();
    pDsDesc->setDepthCompareFunction( MTL::CompareFunction::CompareFunctionLess );
    pDsDesc->setDepthWriteEnabled( true );
    _depthStencilState = device->newDepthStencilState( pDsDesc );
    pDsDesc->release();
}

// -------------------------------- //
//                                  //
//          Compute Command         //
//                                  //
// -------------------------------- //

void Mesh::computeForces(MTL::CommandBuffer* pCmd)
{
    MTL::ComputeCommandEncoder* computeEncoder = pCmd->computeCommandEncoder();
    computeEncoder->setBuffer(_positionBuffer, 0, 0);
    computeEncoder->setBuffer(_forceBuffer, 0, 1);
    computeEncoder->setBuffer(_velocityBuffer, 0, 2);
    computeEncoder->setBuffer(_densityBuffer, 0, 3);
    computeEncoder->setBuffer(_latticeSizeBuffer, 0, 4);
    
    MTL::Size gridSize = MTL::Size(vertices.size(), 1, 1);

    computeEncoder->setComputePipelineState(_forcesPSO);
    MTL::Size threadgroupSize = MTL::Size(256, 1, 1);

    // Encode the compute command.
    computeEncoder->dispatchThreads(gridSize, threadgroupSize);
    
    computeEncoder->endEncoding();
}

// -------------------------------- //
//                                  //
//              Drawing             //
//                                  //
// -------------------------------- //

void Mesh::draw(MTL::RenderCommandEncoder *pEnc, MTL::Buffer* cameraDataBuffer) {
    pEnc->setRenderPipelineState(_renderPSO);
    pEnc->setDepthStencilState(_depthStencilState);

    // Bind vertex data
    pEnc->setVertexBuffer(_positionBuffer, 0, 0);
    pEnc->setVertexBuffer(_normalBuffer, 0, 1);
    pEnc->setVertexBuffer(_forceBuffer, 0, 2);
    pEnc->setVertexBuffer(cameraDataBuffer, 0, 3);

    // Draw primitives
    pEnc->setCullMode(MTL::CullModeBack);
    pEnc->setFrontFacingWinding(MTL::Winding::WindingCounterClockwise);
    pEnc->drawIndexedPrimitives(MTL::PrimitiveType::PrimitiveTypeTriangle,
                                nIndices, MTL::IndexType::IndexTypeUInt32,
                                 _indexBuffer,
                                 0);
}



void Mesh::rotateMesh(float theta, float phi, float psi) {
    // Precompute rotation matrices for theta, phi, and psi
    float cosTheta = cos(theta);
    float sinTheta = sin(theta);
    float cosPhi = cos(phi);
    float sinPhi = sin(phi);
    float cosPsi = cos(psi);
    float sinPsi = sin(psi);

    // Rotation matrix for theta (Y-axis)
    simd_float3x3 rotY = {
        (simd_float3){ cosTheta, 0, sinTheta },
        (simd_float3){ 0,        1, 0        },
        (simd_float3){-sinTheta, 0, cosTheta }
    };

    // Rotation matrix for phi (X-axis)
    simd_float3x3 rotX = {
        (simd_float3){ 1, 0,       0        },
        (simd_float3){ 0, cosPhi, -sinPhi },
        (simd_float3){ 0, sinPhi,  cosPhi }
    };

    // Rotation matrix for psi (Z-axis)
    simd_float3x3 rotZ = {
        (simd_float3){ cosPsi, -sinPsi, 0 },
        (simd_float3){ sinPsi,  cosPsi, 0 },
        (simd_float3){ 0,       0,      1 }
    };
    
    // Combined rotation matrix: rotZ * rotX * rotY
    simd_float3x3 rotationMatrix = simd_mul(simd_mul(rotZ, rotX), rotY);

    // Rotate coordinates
    for (unsigned int i = 0; i < vertices.size(); i++) {
        vertices[i] = simd_mul(rotationMatrix, vertices[i]);
    }
}


void Mesh::scaleMesh(float resolution) {
    simd_float3 minVertex = vertices[0];
    simd_float3 maxVertex = vertices[1];
    for (unsigned int i = 0; i < vertices.size(); i += 3) {
        minVertex.x = fmin(vertices[i].x, minVertex.x);
        minVertex.y = fmin(vertices[i].y, minVertex.y);
        minVertex.z = fmin(vertices[i].z, minVertex.z);
        maxVertex.x = fmax(vertices[i].x, maxVertex.x);
        maxVertex.y = fmax(vertices[i].y, maxVertex.y);
        maxVertex.z = fmax(vertices[i].z, maxVertex.z);
    }
    
    simd_float3 difference = maxVertex - minVertex;
    // We want padding to be proportional to the largest dimension, and equal in all directions:
    float largestDimension = fmax(difference.x, fmax(difference.y, difference.z));
    float paddingMagnitude = (largestDimension / (1 - padding)) - largestDimension;
    simd_float3 boundingBoxShape = {
        difference.x + paddingMagnitude * 2,
        difference.y + paddingMagnitude,
        difference.z + paddingMagnitude
    };
    float currentRes = boundingBoxShape.x * boundingBoxShape.y * boundingBoxShape.z;
    float resScaleFactor = pow(resolution / currentRes, 1.f / 3.f);
    
    simd_float3 floatBoundingBox = {
        boundingBoxShape.x * resScaleFactor,
        boundingBoxShape.y * resScaleFactor,
        boundingBoxShape.z * resScaleFactor
    };
    
    paddingMagnitude *= resScaleFactor;
    
    simd_float3 max = {
        floatBoundingBox.x - paddingMagnitude * 0.5f,
        floatBoundingBox.y - paddingMagnitude * 0.5f,
        floatBoundingBox.z - paddingMagnitude * 0.f
    };
    simd_float3 min = {
        paddingMagnitude * 0.5f,
        paddingMagnitude * 0.5f,
        paddingMagnitude * 0.f
    };
    
    
    simd_float3 goalDifference = max - min;
    simd_float3 ratios = difference / goalDifference;
    float scaleFactor = fmax(ratios.x, fmax(ratios.y, ratios.z));
    
    maxVertex /= scaleFactor;
    minVertex /= scaleFactor;
    
    simd_float3 center = (maxVertex + minVertex) / 2;
    simd_float3 goalCenter = (max + min) / 2;
    simd_float3 translate = goalCenter - center;
    
    for (unsigned int i = 0; i < vertices.size(); i++) {
        vertices[i] = (vertices[i] / scaleFactor) + translate;
    }
    
    meshMinVertex = { (int)ceil(min[0]), (int)ceil(min[1]), (int)ceil(min[2]) };
    meshMaxVertex = { (int)floor(max[0]), (int)floor(max[1]), (int)floor(max[2]) };
    
    simulationBoundingBox.x = (int)floor(floatBoundingBox.x);
    simulationBoundingBox.y = (int)floor(floatBoundingBox.y);
    simulationBoundingBox.z = (int)floor(floatBoundingBox.z);
}
