// Smoothed particle hydrodynamics (SPH).
//
// Each particle is a parcel of fluid. Properties of the fluid at any point are a
// weighted average of the nearby particles, where the weight (the kernel) falls
// smoothly to zero at twice the smoothing length.
//
// Requires particle.glsl.

// The Wendland C2 kernel in two dimensions.
float kernel(float dist)
{
    float h = sim.smoothing_length;
    float q = dist / h;
    if (q >= 2.0F)
    {
        return 0.0F;
    }
    float t = 1.0F - (0.5F * q);
    return (7.0F / (4.0F * 3.14159265F * h * h)) * (t * t * t * t) * ((2.0F * q) + 1.0F);
}

// The gradient of the kernel with respect to the position of the particle at
// the start of `offset` (offset = this particle's position - the other's).
vec2 kernel_gradient(vec2 offset, float dist)
{
    float h = sim.smoothing_length;
    float q = dist / h;
    if (q >= 2.0F || dist < 1.0e-9F)
    {
        return vec2(0.0F);
    }
    float t          = 1.0F - (0.5F * q);
    float derivative = (7.0F / (4.0F * 3.14159265F * h * h)) * (-5.0F * q) * (t * t * t) / h;
    return derivative * (offset / dist);
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

// The average density of the fluid (the particle mass is chosen to make this 1).
const float AVERAGE_DENSITY = 1.0F;

// Weakly compressible: the pressure rises steeply as the fluid is squeezed.
float pressure_from_density(float density)
{
    return sim.sound_speed * sim.sound_speed * (density - sim.rest_density);
}
