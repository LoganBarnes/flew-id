#version 450
#extension GL_GOOGLE_include_directive : require

#include "display.glsl"
#include "colormap.glsl"

layout(location = 0) in vec2 in_position;
layout(location = 1) in vec2 in_velocity;

layout(location = 0) out vec4 frag_color;

void main()
{
    vec3 color = (display.particle_mode == PARTICLE_SPEED)
               ? viridis(length(in_velocity) / display.max_speed)
               : vec3(1.0F);

    frag_color = vec4(color, display.particle_brightness);

    gl_Position = display.clip_from_world * vec4(in_position, 0.0F, 1.0F);

    // World-space diameter converted to pixels (never smaller than one pixel).
    float pixels_per_unit = abs(display.clip_from_world[0][0]) * display.viewport_size.x * 0.5F;
    gl_PointSize = max(1.0F, display.diameter * pixels_per_unit);
}
