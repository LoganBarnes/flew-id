// Hash based random numbers.

// https://www.pcg-random.org/
uint pcg_hash(uint value)
{
    uint state = (value * 747796405U) + 2891336453U;
    uint word  = ((state >> ((state >> 28U) + 4U)) ^ state) * 277803737U;
    return (word >> 22U) ^ word;
}

// Uniform in [0, 1). Advances the seed.
float random_float(inout uint seed)
{
    seed = pcg_hash(seed);
    return float(seed >> 8U) * (1.0F / 16777216.0F);
}

// Two independent normally distributed values (Box-Muller transform).
vec2 random_normal(inout uint seed)
{
    float u1 = max(random_float(seed), 1.0e-7F);
    float u2 = random_float(seed);

    float magnitude = sqrt(-2.0F * log(u1));
    float angle     = 6.28318530718F * u2;

    return magnitude * vec2(cos(angle), sin(angle));
}
