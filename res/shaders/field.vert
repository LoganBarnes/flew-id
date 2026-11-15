#version 450
#extension GL_GOOGLE_include_directive : require

// A rectangle covering the tunnel (drawn as a four vertex triangle strip).

#include "display.glsl"

layout(location = 0) out vec2 world_position;

void main()
{
    vec2 corner = vec2(float(gl_VertexIndex & 1), float(gl_VertexIndex >> 1));

    world_position = corner * display.tunnel_size;
    gl_Position    = display.clip_from_world * vec4(world_position, 0.0F, 1.0F);
}
