// Smoothed particle hydrodynamics (SPH).
//
// Each particle is a parcel of fluid. Properties of the fluid at any point are a
// weighted average of the nearby particles, where the weight (the kernel) falls
// smoothly to zero at twice the smoothing length.
//
// Requires particle.glsl.

// The Wendland C2 kernel in two dimensions:
//
//   W(r) = (7 / (4 pi h^2)) t^4 (2 q + 1),   q = r / h,   t = 1 - q / 2   (zero when q >= 2)
//
// The neighbor loops evaluate it in pieces so constants can be applied once per particle.

// The normalization of the kernel, 7 / (4 pi h^2).
float kernel_scale()
{
    float h = sim.smoothing_length;
    return 7.0F / (4.0F * 3.14159265F * h * h);
}

// W(r) / kernel_scale() for a squared distance within the support (r < 2 h).
float kernel_shape(float dist_sq)
{
    float q = sqrt(dist_sq) / sim.smoothing_length;
    float t = 1.0F - (0.5F * q);
    return (t * t) * (t * t) * ((2.0F * q) + 1.0F);
}

// The gradient of the kernel is gradient_scale() * t^3 * offset, where offset is this
// particle's position minus the other's. (dW/dr = -5 q t^3 kernel_scale() / h, and the
// direction offset / r cancels the q.)
float gradient_scale()
{
    float h = sim.smoothing_length;
    return -5.0F * kernel_scale() / (h * h);
}

// t^3 for a squared distance within the support (r < 2 h).
float gradient_shape(float dist_sq)
{
    float t = 1.0F - (0.5F * sqrt(dist_sq) / sim.smoothing_length);
    return t * t * t;
}

// Solid walls are treated as if they were filled with fluid at the average density.
// These tables hold the integral of the kernel over the part of its support that is
// beyond a flat wall, for wall distances of 0 to 2 smoothing lengths (9 samples):
//  - WALL_FRACTION is the integral itself (the wall's share of the density), and
//  - WALL_SLOPE is how quickly it decreases with distance (used for the wall's force).
const float WALL_FRACTION[9] = float[9](
    0.50000F, 0.32045F, 0.17602F, 0.08011F, 0.02837F, 0.00690F, 0.00086F, 0.00002F, 0.0F
);
const float WALL_SLOPE[9] = float[9](
    0.74272F, 0.66398F, 0.48373F, 0.28863F, 0.13633F, 0.04645F, 0.00907F, 0.00047F, 0.0F
);

float wall_table(float[9] table, float wall_distance)
{
    float s = clamp(wall_distance / sim.smoothing_length, 0.0F, 2.0F) * 4.0F;
    int   i = min(int(s), 7);
    return mix(table[i], table[i + 1], s - float(i));
}

// The fraction of a particle's density supplied by a wall `wall_distance` away.
float wall_fraction(float wall_distance)
{
    return wall_table(WALL_FRACTION, wall_distance);
}

// The magnitude of the gradient of wall_fraction (it points towards the wall).
float wall_gradient(float wall_distance)
{
    return wall_table(WALL_SLOPE, wall_distance) / sim.smoothing_length;
}

// Grid cells are half the kernel's support wide (see `sph_cell_subdivision` in
// particle_system.cpp), so neighbors are up to this many cells away in each direction.
const int SPH_CELL_REACH = 2;

// Runs NEIGHBOR(other, shift) for every particle within SPH_CELL_REACH cells of `coords`
// in grid row `y`, except particle `self_index`. Cells in a row are contiguous in the
// sorted buffer so each run of cells is processed as a single range. When `flowing`,
// the ends of the tunnel are connected: cells beyond one end wrap around to the other,
// and `shift` is subtracted from the x offset to the neighbor.
#define FOR_EACH_NEIGHBOR_IN_ROW(coords, y, self_index, flowing, NEIGHBOR)                     \
    {                                                                                          \
        int  lower_x = coords.x - SPH_CELL_REACH;                                              \
        int  upper_x = coords.x + SPH_CELL_REACH;                                              \
        uint first   = row_start(max(lower_x, 0), y);                                          \
        uint last    = grid[cell_index(ivec2(min(upper_x, sim.grid_size.x - 1), y))];         \
        for (uint other = first; other < last; ++other)                                        \
        {                                                                                      \
            if (other != self_index)                                                           \
            {                                                                                  \
                NEIGHBOR(other, 0.0F);                                                         \
            }                                                                                  \
        }                                                                                      \
        if (flowing && lower_x < 0)                                                            \
        {                                                                                      \
            first = row_start(sim.grid_size.x + lower_x, y);                                   \
            last  = grid[cell_index(ivec2(sim.grid_size.x - 1, y))];                           \
            for (uint other = first; other < last; ++other)                                    \
            {                                                                                  \
                NEIGHBOR(other, -sim.tunnel_size.x);                                           \
            }                                                                                  \
        }                                                                                      \
        if (flowing && upper_x >= sim.grid_size.x)                                             \
        {                                                                                      \
            first = row_start(0, y);                                                           \
            last  = grid[cell_index(ivec2(upper_x - sim.grid_size.x, y))];                     \
            for (uint other = first; other < last; ++other)                                    \
            {                                                                                  \
                NEIGHBOR(other, sim.tunnel_size.x);                                            \
            }                                                                                  \
        }                                                                                      \
    }

// The index of the first sorted particle in cell (x, y).
uint row_start(int x, int y)
{
    uint cell = cell_index(ivec2(x, y));
    return (cell == 0U) ? 0U : grid[cell - 1U];
}

// The average density of the fluid (the particle mass is chosen to make this 1).
const float AVERAGE_DENSITY = 1.0F;

// Weakly compressible: the pressure rises steeply as the fluid is squeezed.
float pressure_from_density(float density)
{
    return sim.sound_speed * sim.sound_speed * (density - sim.rest_density);
}
