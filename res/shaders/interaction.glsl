// The particle-particle interaction model.
//
// Particles are equal (unit) mass soft disks. Every pair of overlapping particles:
//  - is pushed apart by a spring force proportional to the overlap (stiffness),
//  - loses some of its approach/separation speed (damping), and
//  - shares some tangential velocity (viscosity).
//
// The spring force is conservative so with no damping or viscosity the total energy
// is conserved. Each particle only computes the force on itself. The other particle
// computes the equal and opposite force, so momentum is always conserved.
void interact(inout vec2 acceleration, Particle self, Particle other)
{
    float diameter = 2.0F * sim.radius;

    vec2  offset      = self.position - other.position;
    float distance_sq = dot(offset, offset);

    if (distance_sq >= (diameter * diameter) || distance_sq < 1.0e-12F)
    {
        return;
    }

    float dist    = sqrt(distance_sq);
    vec2  normal  = offset / dist;
    float overlap = diameter - dist;

    vec2  relative_velocity = self.velocity - other.velocity;
    float normal_speed      = dot(relative_velocity, normal);
    vec2  tangent_velocity  = relative_velocity - (normal_speed * normal);

    acceleration += (sim.stiffness * overlap) * normal;
    acceleration -= (sim.damping * normal_speed) * normal;
    acceleration -= sim.viscosity * tangent_velocity;
}
