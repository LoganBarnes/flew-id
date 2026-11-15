#version 450

// A rectangle covering the tunnel (drawn as a four vertex triangle strip).

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

layout(location = 0) out vec2 world_position;

void main()
{
    vec2 corner = vec2(float(gl_VertexIndex & 1), float(gl_VertexIndex >> 1));

    world_position = corner * display.tunnel_size;
    gl_Position    = display.clip_from_world * vec4(world_position, 0.0F, 1.0F);
}
