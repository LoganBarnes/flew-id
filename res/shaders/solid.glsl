// Sampling of the solids' signed distance field.
//
// Requires `solids` (an array of vec4 texels in row major order where x is the
// signed distance at the center of the texel and yz is the direction away from
// the nearest surface) to be declared before this file is included.

// Bilinearly interpolated texel at a world position.
vec4 sample_solids(vec2 position, vec2 tunnel_size, ivec2 solid_size)
{
    vec2  coords = (position / tunnel_size * vec2(solid_size)) - 0.5F;
    ivec2 lower  = ivec2(floor(coords));
    vec2  t      = coords - vec2(lower);

    ivec2 max_coords = solid_size - ivec2(1);
    ivec2 c00 = clamp(lower, ivec2(0), max_coords);
    ivec2 c11 = clamp(lower + ivec2(1), ivec2(0), max_coords);

    vec4 v00 = solids[(c00.y * solid_size.x) + c00.x];
    vec4 v10 = solids[(c00.y * solid_size.x) + c11.x];
    vec4 v01 = solids[(c11.y * solid_size.x) + c00.x];
    vec4 v11 = solids[(c11.y * solid_size.x) + c11.x];

    return mix(mix(v00, v10, t.x), mix(v01, v11, t.x), t.y);
}
