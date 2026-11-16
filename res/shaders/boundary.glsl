// Everything that happens to a particle at the edges of the fluid: the ends of the tunnel
// (with or without flow), the top and bottom walls, and the solids.
//
// Requires particle.glsl, random.glsl, and solid.glsl.

void apply_boundaries(uint index, inout vec2 position, inout vec2 velocity)
{
    bool flowing = (sim.flow_speed > 0.0F);
    bool sph     = (sim.model == MODEL_SPH);

    vec2 lower = vec2(sim.radius);
    vec2 upper = sim.tunnel_size - vec2(sim.radius);

    if (flowing)
    {
        uint seed = pcg_hash(index ^ pcg_hash(sim.step_index));

        // Particles leaving one end of the tunnel enter the other.
        bool left_tunnel = (position.x < 0.0F) || (position.x >= sim.tunnel_size.x);
        position.x = mod(position.x, sim.tunnel_size.x);

        // Reset particles near the end(s) of the tunnel to the incoming flow
        // conditions. This drives the flow, removes the wake of the solids before
        // it re-enters the tunnel, and keeps the temperature of the fluid constant.
        float reset_rate = 0.0F;

        if ((sim.boundary_flags & BUFFERS_FLAG) != 0U)
        {
            // Buffer zones at both ends. The outlet buffer absorbs wakes and pressure
            // waves before they reach the end of the tunnel and the inlet buffer
            // delivers a uniform flow. Resets become gradually stronger towards
            // the ends so the start of a buffer doesn't act like a wall.
            float end_distance = min(position.x, sim.tunnel_size.x - position.x);

            if (end_distance < sim.inflow_width)
            {
                float strength = 1.0F - (end_distance / sim.inflow_width);
                reset_rate = sim.inflow_rate * strength * strength;
            }
        }
        else if (position.x < sim.inflow_width)
        {
            reset_rate = sim.inflow_rate;
        }

        vec2 flow_velocity = vec2(sim.flow_speed, 0.0F);
        bool respawn       = left_tunnel && ((sim.boundary_flags & RESPAWN_FLAG) != 0U);

        if (sph)
        {
            // SPH particles have no random motion so they are smoothly
            // steered towards the incoming flow instead of being replaced.
            float blend = respawn ? 1.0F : clamp(reset_rate * sim.delta_time, 0.0F, 1.0F);
            velocity = mix(velocity, flow_velocity, blend);
        }
        else if (respawn || random_float(seed) < (reset_rate * sim.delta_time))
        {
            // The particle becomes a new particle of the undisturbed flow.
            velocity = flow_velocity + (random_normal(seed) * sim.thermal_speed * 0.70710678F);
        }
    }
    else
    {
        // Reflect off the ends of the tunnel.
        if (position.x < lower.x) { position.x = (2.0F * lower.x) - position.x; velocity.x = abs(velocity.x); }
        if (position.x > upper.x) { position.x = (2.0F * upper.x) - position.x; velocity.x = -abs(velocity.x); }
        position.x = clamp(position.x, lower.x, upper.x);
    }

    // Reflect off the top and bottom of the tunnel.
    if (position.y < lower.y) { position.y = (2.0F * lower.y) - position.y; velocity.y = abs(velocity.y); }
    if (position.y > upper.y) { position.y = (2.0F * upper.y) - position.y; velocity.y = -abs(velocity.y); }
    position.y = clamp(position.y, lower.y, upper.y);

    // Collide with the solids.
    vec4  solid    = sample_solids(position, sim.tunnel_size, sim.solid_size);
    float distance = solid.x;

    if (distance < sim.radius)
    {
        vec2 normal = normalize(solid.yz + vec2(0.0F, 1.0e-12F));

        // Move back to the surface.
        position += (sim.radius - distance) * normal;

        float normal_speed     = dot(velocity, normal);
        vec2  tangent_velocity = velocity - (normal_speed * normal);

        if (normal_speed < 0.0F)
        {
            normal_speed = -sim.wall_restitution * normal_speed;
        }
        velocity = (normal_speed * normal) + ((1.0F - sim.wall_friction) * tangent_velocity);
    }
}
