// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#pragma once

// project
#include "ltb/utils/result.hpp"
#include "ltb/utils/types.hpp"

// external
#include <glm/glm.hpp>

// standard
#include <functional>
#include <span>
#include <vector>

namespace ltb::flew
{

/// \brief A single fluid particle. Matches the layout of `Particle` in `particle.glsl`.
struct Particle
{
    glm::vec2 position = { };
    glm::vec2 velocity = { };
};

/// \brief The rectangular region `[0, size.x] x [0, size.y]` containing the fluid.
struct Tunnel
{
    glm::vec2 size = { 4.0F, 2.0F };
};

struct ParticleFillSettings
{
    uint32 particle_count = 200'000U;
    /// \brief The fraction of the tunnel's area covered by particles. This
    ///        and the particle count determine the radius of the particles.
    float32 packing_fraction = 0.4F;
    /// \brief The root mean square speed of the random (thermal) motion of the particles.
    float32 thermal_speed = 0.5F;
    /// \brief The average initial velocity of the particles.
    glm::vec2 mean_velocity = { };
    uint32  seed          = 1U;
};

/// \brief The particle radius required to cover `packing_fraction` of the tunnel.
[[nodiscard( "Pure function" )]]
auto particle_radius( Tunnel const& tunnel, ParticleFillSettings const& settings ) -> float32;

/// \brief Returns the signed distance from a point to the nearest solid (negative inside).
using DistanceFunction = std::function< float32( glm::vec2 const& ) >;

/// \brief Fills the tunnel with particles on a jittered grid so no
///        two particles overlap and none are inside a solid.
[[nodiscard( "Pure function" )]]
auto fill_tunnel(
    Tunnel const&               tunnel,
    ParticleFillSettings const& settings,
    DistanceFunction const&     solid_distance = nullptr
) -> std::vector< Particle >;

/// \brief Summary values used to check the health of a simulation.
struct ParticleStats
{
    uint64 count = 0UZ;
    /// \brief Particles with a non-finite position or velocity.
    uint64 invalid_count = 0UZ;
    /// \brief Particles whose center is outside the tunnel.
    uint64 out_of_bounds_count = 0UZ;

    /// \brief Sum of `0.5 * speed^2` (unit mass).
    float64   kinetic_energy = 0.0;
    glm::vec2 momentum       = { };
    float32   max_speed      = 0.0F;

    /// \brief The deepest overlap between two particles as a fraction of the diameter.
    float32 max_overlap = 0.0F;
    /// \brief The number of overlapping pairs.
    uint64 overlapping_pairs = 0UZ;
};

[[nodiscard( "Pure function" )]]
auto compute_stats( std::span< Particle const > particles, Tunnel const& tunnel, float32 radius )
    -> ParticleStats;

/// \brief A single line description of the stats.
[[nodiscard( "Pure function" )]]
auto to_string( ParticleStats const& stats ) -> std::string;

/// \brief Verifies a simulation with no damping is still healthy: no particles were lost
///        or became invalid, energy was (roughly) conserved, and no particles passed
///        through each other.
///
/// When `driven` is true energy is being added and removed (e.g. by the inflow of a
/// wind tunnel) so it is only required to stay within an order of magnitude.
auto check_stats( ParticleStats const& initial, ParticleStats const& current, bool driven = false )
    -> utils::Result< void >;

} // namespace ltb::flew
