#version 450
#extension GL_GOOGLE_include_directive : require

// Draws the tunnel background and the solids.

// Matches `ltb::flew::DisplayUniforms`.
layout(std140, binding = 0) uniform DisplayUbo
{
    mat4  clip_from_world;
    vec2  viewport_size;
    float diameter;
    float max_speed;
    vec2  tunnel_size;
    ivec2 solid_size;
} display;

layout(std430, binding = 1) readonly buffer Solids
{
    vec4 solids[];
};

#include "solid.glsl"

layout(location = 0) in vec2 world_position;

layout(location = 0) out vec4 out_color;

void main()
{
    const vec3 background_color = vec3(0.04F, 0.04F, 0.06F);
    const vec3 solid_color      = vec3(0.62F, 0.62F, 0.66F);

    float distance = sample_solids(world_position, display.tunnel_size, display.solid_size).x;

    // Blend over roughly one pixel at the surface.
    float pixel   = max(fwidth(distance), 1.0e-6F);
    float outside = smoothstep(-0.5F * pixel, 0.5F * pixel, distance);

    out_color = vec4(mix(solid_color, background_color, outside), 1.0F);
}
