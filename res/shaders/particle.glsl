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
    // The length of the region(s) at the end(s) of the tunnel where particles are randomly
    // reset to the incoming flow conditions, and how often (1/s) each particle is reset.
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

    // RESPAWN_FLAG: a particle leaving either end of the tunnel is replaced by a new
    // particle of the undisturbed incoming flow (as if the tunnel continued far beyond
    // both ends) instead of keeping its velocity when it re-enters at the other end.
    //
    // BUFFERS_FLAG: particles are reset in buffer zones at both ends of the tunnel
    // (more often the closer they are to the end) instead of only near the inlet.
    //
    // SHIFTING_FLAG: SPH particles are nudged towards an even spacing (see sph_integrate.comp).
    uint flags;

    // Which fluid model is being simulated (MODEL_GAS or MODEL_SPH).
    uint model;
    // SPH: the speed of sound, which sets how strongly the fluid resists compression.
    float sound_speed;
    // SPH: the kernel's smoothing length. Particles interact within twice this distance.
    float smoothing_length;
    // SPH: the mass of each particle (the average density of the fluid is 1).
    float particle_mass;
    // SPH: the density at which the pressure is zero. Slightly less than
    // the average density so the fluid is always under a little pressure.
    float rest_density;
} sim;

// A gas of colliding disks with random thermal motion.
const uint MODEL_GAS = 0U;
// Smoothed particle hydrodynamics: each particle is a parcel of fluid.
const uint MODEL_SPH = 1U;

const uint RESPAWN_FLAG = 1U;
const uint BUFFERS_FLAG = 2U;
const uint SHIFTING_FLAG = 4U;

// The number of values summed by a single workgroup of the prefix sum's up sweep.
const uint SCAN_BLOCK_SIZE = 1024U;

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

// SPH: the density of the fluid at each particle (in sorted order, matching particles_b).
layout(std430, binding = 5) buffer Densities
{
    float densities[];
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
