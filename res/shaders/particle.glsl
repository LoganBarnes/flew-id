// Shared definitions for the particle simulation shaders.

// Matches `ltb::flew::Particle`.
struct Particle
{
    vec2 position;
    vec2 velocity;
};

// Matches `ltb::flew::StepConstants`.
layout(push_constant, std430) uniform StepConstants
{
    vec2  tunnel_size;
    float delta_time;
    float radius;

    ivec2 grid_size;
    uint  particle_count;
    // Spring constant of the repulsion between overlapping particles.
    float stiffness;

    // Rate at which touching particles share tangential velocity.
    float viscosity;
    // Rate at which touching particles lose relative normal velocity.
    float damping;
    // Prefix sum pass: the level being processed and its parent level.
    uint  scan_child_offset;
    uint  scan_child_count;

    uint  scan_parent_offset;
    // Incremented every step (used to generate random numbers).
    uint  step_index;
    ivec2 solid_size;

    // Speed of the flow entering the left side of the tunnel. When zero the tunnel is a
    // closed box, otherwise particles leaving the right side re-enter on the left.
    float flow_speed;
    // Root mean square speed of the random motion given to entering particles.
    float thermal_speed;
    // Particles with x < inflow_width are randomly reset to the incoming flow conditions.
    float inflow_width;
    float inflow_rate;

    // Fraction of tangential velocity lost when touching a solid (0 = slip, 1 = no-slip).
    float wall_friction;
    // Fraction of normal speed kept when bouncing off a solid.
    float wall_restitution;

    // The flow fields are stored on a coarser grid. Each field cell
    // covers `field_scale` x `field_scale` cells of the particle grid.
    ivec2 field_size;
    uint  field_scale;
    // How much of each new field measurement is blended into the running average [0, 1].
    float field_blend;
} sim;

// The number of cells summed by a single invocation of the prefix sum passes.
const uint SCAN_BLOCK_SIZE = 256U;

// All compute passes share the same bindings.
//  - particles_a holds the current state at the start and end of every step.
//  - particles_b holds the particles sorted by grid cell during a step.
//  - grid holds per-cell counters (and the prefix sum hierarchy after them).
layout(std430, binding = 0) buffer ParticlesA
{
    Particle particles_a[];
};

layout(std430, binding = 1) buffer ParticlesB
{
    Particle particles_b[];
};

layout(std430, binding = 2) buffer Grid
{
    uint grid[];
};

// x: signed distance to the nearest solid, yz: direction away from it.
layout(std430, binding = 3) readonly buffer Solids
{
    vec4 solids[];
};

// Time averaged flow properties of each field cell.
// x: packing fraction, yz: mean velocity, w: temperature (mean squared thermal speed).
layout(std430, binding = 4) buffer Fields
{
    vec4 fields[];
};

ivec2 cell_coords(vec2 position)
{
    ivec2 coords = ivec2(floor(position * vec2(sim.grid_size) / sim.tunnel_size));
    return clamp(coords, ivec2(0), sim.grid_size - ivec2(1));
}

uint cell_index(ivec2 coords)
{
    return (uint(coords.y) * uint(sim.grid_size.x)) + uint(coords.x);
}
