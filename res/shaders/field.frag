#version 450
#extension GL_GOOGLE_include_directive : require

// Draws the tunnel background, colored by a property of the flow, and the solids.

#include "display.glsl"
#include "colormap.glsl"

layout(std430, binding = 1) readonly buffer Solids
{
    vec4 solids[];
};

// x: packing fraction, yz: mean velocity, w: temperature.
layout(std430, binding = 2) readonly buffer Fields
{
    vec4 fields[];
};

#include "solid.glsl"

layout(location = 0) in vec2 world_position;

layout(location = 0) out vec4 out_color;

// Bilinearly interpolated flow properties at a world position.
vec4 sample_fields(vec2 position)
{
    vec2  coords = (position / display.tunnel_size * vec2(display.field_size)) - 0.5F;
    ivec2 lower  = ivec2(floor(coords));
    vec2  t      = coords - vec2(lower);

    ivec2 max_coords = display.field_size - ivec2(1);
    ivec2 c00 = clamp(lower, ivec2(0), max_coords);
    ivec2 c11 = clamp(lower + ivec2(1), ivec2(0), max_coords);

    vec4 v00 = fields[(c00.y * display.field_size.x) + c00.x];
    vec4 v10 = fields[(c00.y * display.field_size.x) + c11.x];
    vec4 v01 = fields[(c11.y * display.field_size.x) + c00.x];
    vec4 v11 = fields[(c11.y * display.field_size.x) + c11.x];

    return mix(mix(v00, v10, t.x), mix(v01, v11, t.x), t.y);
}

// The pressure of a gas of disks: density * temperature, increased by the space the
// disks themselves take up (scaled particle theory equation of state for hard disks).
float pressure(float density, float temperature)
{
    float free_fraction = max(1.0F - density, 0.05F);
    return density * temperature / (free_fraction * free_fraction);
}

vec3 field_color(vec2 position)
{
    vec4 field = sample_fields(position);

    switch (display.field_mode)
    {
        case FIELD_SPEED:
            return viridis(length(field.yz) / display.max_speed);

        case FIELD_PRESSURE:
        {
            float reference = pressure(display.reference_density, display.reference_temperature);
            return diverging(((pressure(field.x, field.w) / reference) - 1.0F) / display.field_range);
        }

        case FIELD_DENSITY:
            return diverging(((field.x / display.reference_density) - 1.0F) / display.field_range);

        case FIELD_VORTICITY:
        {
            // The curl of the velocity. Positive is counter-clockwise rotation.
            vec2 h = display.tunnel_size / vec2(display.field_size);

            float dvy_dx = (sample_fields(position + vec2(h.x, 0.0F)).z
                          - sample_fields(position - vec2(h.x, 0.0F)).z) / (2.0F * h.x);
            float dvx_dy = (sample_fields(position + vec2(0.0F, h.y)).y
                          - sample_fields(position - vec2(0.0F, h.y)).y) / (2.0F * h.y);

            // Scaled so a speed change of max_speed over a tenth of the tunnel height is 1.
            float scale = display.max_speed / (0.1F * display.tunnel_size.y);
            return diverging((dvy_dx - dvx_dy) / (scale * display.field_range * 10.0F));
        }

        case FIELD_TEMPERATURE:
            return diverging(((field.w / display.reference_temperature) - 1.0F) / display.field_range);
    }

    return vec3(0.04F, 0.04F, 0.06F);
}

void main()
{
    const vec3 solid_color = vec3(0.62F, 0.62F, 0.66F);

    float distance = sample_solids(world_position, display.tunnel_size, display.solid_size).x;

    // Blend over roughly one pixel at the surface.
    float pixel   = max(fwidth(distance), 1.0e-6F);
    float outside = smoothstep(-0.5F * pixel, 0.5F * pixel, distance);

    vec3 color = field_color(world_position);

    // Mark the buffer zones (where the flow is artificially reset) with a
    // slight tint and a line where they meet the rest of the tunnel.
    float end_distance = min(world_position.x, display.tunnel_size.x - world_position.x);
    if (end_distance < display.buffer_width)
    {
        float line_width = 1.5F * fwidth(world_position.x);
        float on_line    = step(display.buffer_width - line_width, end_distance);

        color = mix(color, vec3(0.5F), 0.12F + (0.4F * on_line));
    }

    out_color = vec4(mix(solid_color, color, outside), 1.0F);
}
