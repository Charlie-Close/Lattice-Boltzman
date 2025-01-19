//
//  render.metal
//  Lattice Boltzman
//
//  Created by Charlie Close on 08/12/2024.
//

#include <metal_stdlib>
#include "../../Parameters/SimulationConstants.h"
#include "./Helpers/Interpolate/Interpolate.h"
#include "../../Parameters/Parameters.h"
using namespace metal;

struct MeshVertOut {
    float4 position [[position]];  // Position in clip space
    float3 fragPosition;           // World-space position
    float3 fragNormal;             // World-space normal
    half3 colour;
};

struct PointVertOut {
    float4 position [[position]];
    float density;
    float pointSize [[point_size]];
};


half3 momentumColour(float momentum) {
    float normalised = clamp(momentum * 5.f / airSpeed, 0.f, 1.f);
    half b = clamp(1 - 1.8 * abs(0.f - normalised), 0.f, 1.f);
    half g = clamp(1 - 1.8 * abs(0.5f - normalised), 0.f, 1.f);
    half r = clamp(1 - 1.8 * abs(1.f - normalised), 0.f, 1.f);
    
    return { r, g, b };
}

half3 densityColour(float density) {
    float normalised = clamp((density - 1.f + 0.1 * airSpeed) * 8.f / airSpeed, 0.f, 1.f);
    half b = clamp(1 - 1.8 * abs(0.f - normalised), 0.f, 1.f);
    half g = clamp(1 - 1.8 * abs(0.5f - normalised), 0.f, 1.f);
    half r = clamp(1 - 1.8 * abs(1.f - normalised), 0.f, 1.f);
    
    return { r, g, b };
}


MeshVertOut vertex vertexMain(device const float3* positions [[buffer(0)]],
                              device const float3* normals [[buffer(1)]],
                              device const float* forces [[buffer(2)]],
                              constant float4x4& cameraMatrix [[buffer(3)]],
                              uint vertexId [[vertex_id]]) {
    MeshVertOut o;
    float force = forces[vertexId];
//    o.colour = momentumColour(force);
    o.colour = densityColour(force);

    // Transform position
    float4 worldPosition = float4(positions[vertexId], 1.0);
    o.position = cameraMatrix * worldPosition;

    // Pass data to fragment shader
    o.fragPosition = positions[vertexId];
    o.fragNormal = normalize(normals[vertexId]);  // Ensure normal is normalized

    return o;
}

vertex PointVertOut vertexPoint(device const float3* positions [[buffer(0)]],
                       device const float* densities [[buffer(1)]],
                       device const float* velocities [[buffer(2)]],
                       constant float4x4& cameraMatrix [[buffer(3)]],
                       uint vertexId [[vertex_id]]) {
    PointVertOut o;
    float4 worldPosition = float4(positions[vertexId], 1.0);
    o.position = cameraMatrix * worldPosition;
    o.density = densities[vertexId];

    // Set a defined point size (in pixels)
    o.pointSize = particleSize; // or any desired size

    return o;
}

half4 fragment fragmentMain( MeshVertOut in [[stage_in]] )
{
    // assume light coming from (front-top-right)
    float3 l = normalize(float3( 1.0, 1.0, 0.8 ));
    float3 n = normalize( in.fragNormal );

    float ndotl = saturate( dot( n, l ) );
    return half4( in.colour * 0.1 + in.colour * ndotl, 1.0 );
}

half4 fragment fragmentPoint(PointVertOut in [[stage_in]])
{
    return half4( densityColour(in.density), 1.0 );
}
