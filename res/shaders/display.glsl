// Uniforms shared by the shaders that draw the tunnel. Matches `ltb::flew::DisplayUniforms`.
layout(std140, binding = 0) uniform DisplayUbo
{
    mat4  clip_from_world;
    vec2  viewport_size;
    float diameter;
    float max_speed;
    vec2  tunnel_size;
    ivec2 solid_size;
    ivec2 field_size;
    // What the background shows (see FIELD_* in display.glsl).
    int   field_mode;
    // The largest relative difference (or vorticity) shown by the two sided color scales.
    float field_range;
    // The packing fraction and temperature of the undisturbed fluid.
    float reference_density;
    float reference_temperature;
    // How the particles are colored (see PARTICLE_* in display.glsl).
    int   particle_mode;
    float particle_brightness;
    // The length of the buffer zone at each end of the tunnel (zero if there are none).
    float buffer_width;
    // SPH: the dynamic pressure of the incoming flow (0.5 * density * speed^2). When
    // not zero the pressure is shown as a pressure coefficient instead of being
    // estimated from the density and temperature of a gas.
    float dynamic_pressure;
} display;

const int FIELD_NONE        = 0;
const int FIELD_SPEED       = 1;
const int FIELD_PRESSURE    = 2;
const int FIELD_DENSITY     = 3;
const int FIELD_VORTICITY   = 4;
const int FIELD_TEMPERATURE = 5;

const int PARTICLE_SPEED = 0;
const int PARTICLE_PLAIN = 1;
