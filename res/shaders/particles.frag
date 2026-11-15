#version 450

layout(location = 0) in vec4 frag_color;

layout(location = 0) out vec4 out_color;

void main()
{
    // Draw each point as a disk.
    vec2 offset = (gl_PointCoord * 2.0F) - 1.0F;
    if (dot(offset, offset) > 1.0F)
    {
        discard;
    }
    out_color = frag_color;
}
