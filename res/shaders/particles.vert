#version 450
#extension GL_GOOGLE_include_directive : require

#include "colormap.glsl"

layout(location = 0) in vec2 in_position;
layout(location = 1) in vec2 in_velocity;

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

layout(location = 0) out vec4 frag_color;

void main()
{
    frag_color = vec4(viridis(length(in_velocity) / display.max_speed), 1.0F);

    gl_Position = display.clip_from_world * vec4(in_position, 0.0F, 1.0F);

    // World-space diameter converted to pixels (never smaller than one pixel).
    float pixels_per_unit = abs(display.clip_from_world[0][0]) * display.viewport_size.x * 0.5F;
    gl_PointSize = max(1.0F, display.diameter * pixels_per_unit);
}
