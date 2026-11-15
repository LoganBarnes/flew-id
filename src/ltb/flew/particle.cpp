// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "ltb/flew/particle.hpp"

// external
#include <glm/gtc/constants.hpp>

// external
#include <spdlog/fmt/fmt.h>

// standard
#include <algorithm>
#include <cmath>
#include <random>

namespace ltb::flew
{

auto particle_radius( Tunnel const& tunnel, ParticleFillSettings const& settings ) -> float32
{
    auto const count = static_cast< float32 >( std::max( 1U, settings.particle_count ) );
    auto const area  = tunnel.size.x * tunnel.size.y;

    return std::sqrt( ( settings.packing_fraction * area ) / ( count * glm::pi< float32 >( ) ) );
}

auto fill_tunnel(
    Tunnel const&               tunnel,
    ParticleFillSettings const& settings,
    DistanceFunction const&     solid_distance
) -> std::vector< Particle >
{
    auto const radius = particle_radius( tunnel, settings );

    auto particles = std::vector< Particle >( settings.particle_count );
    if ( particles.empty( ) )
    {
        return particles;
    }

    auto const aspect = tunnel.size.x / tunnel.size.y;

    // Find a grid with the same aspect ratio as the tunnel that has
    // at least as many cells outside of the solids as there are particles.
    auto cell       = glm::vec2{ };
    auto free_cells = std::vector< glm::vec2 >{ };

    for ( auto scale = 1.0F; free_cells.size( ) < particles.size( ); scale *= 1.05F )
    {
        auto const count = static_cast< float32 >( settings.particle_count ) * scale;
        auto const cols  = static_cast< uint32 >( std::ceil( std::sqrt( count * aspect ) ) );
        auto const rows  = static_cast< uint32 >( std::ceil( count / static_cast< float32 >( cols ) ) );

        cell = tunnel.size / glm::vec2( glm::uvec2{ cols, rows } );

        free_cells.clear( );
        for ( auto row = 0U; row < rows; ++row )
        {
            for ( auto col = 0U; col < cols; ++col )
            {
                auto const center = ( glm::vec2( glm::uvec2{ col, row } ) + 0.5F ) * cell;
                // Keep the whole cell clear of any solids.
                if ( !solid_distance || solid_distance( center ) > glm::length( cell ) )
                {
                    free_cells.push_back( center );
                }
            }
        }
    }

    // Each particle may move within its cell as long as it stays fully inside.
    auto const jitter = glm::max( glm::vec2( 0.0F ), ( cell * 0.5F ) - radius );

    auto rand_gen  = std::mt19937{ settings.seed };
    auto unit_dist = std::uniform_real_distribution( -1.0F, 1.0F );
    // Each velocity component is normally distributed (a 2D Maxwell-Boltzmann distribution).
    auto velocity_dist = std::normal_distribution( 0.0F, settings.thermal_speed / glm::root_two< float32 >( ) );

    // Spread the particles evenly over the free cells.
    auto const cells_per_particle = static_cast< float64 >( free_cells.size( ) )
                                  / static_cast< float64 >( particles.size( ) );

    for ( auto index = 0UZ; index < particles.size( ); ++index )
    {
        auto const cell_index = static_cast< size_t >( static_cast< float64 >( index ) * cells_per_particle );
        auto const offset = glm::vec2{ unit_dist( rand_gen ), unit_dist( rand_gen ) } * jitter;

        particles[ index ] = Particle{
            .position = free_cells[ cell_index ] + offset,
            .velocity = settings.mean_velocity
                      + glm::vec2{ velocity_dist( rand_gen ), velocity_dist( rand_gen ) },
        };
    }

    return particles;
}

auto compute_stats(
    std::span< Particle const > const particles,
    Tunnel const&                     tunnel,
    float32 const                     radius
) -> ParticleStats
{
    auto stats = ParticleStats{ .count = particles.size( ) };

    // Bin the particles into a grid of diameter sized cells to find overlapping pairs.
    auto const diameter  = 2.0F * radius;
    auto const grid_size = glm::max( glm::ivec2( 1 ), glm::ivec2( tunnel.size / diameter ) );
    auto const to_cell   = [ & ]( glm::vec2 const& position ) {
        auto const coords = glm::ivec2( glm::floor( position * glm::vec2( grid_size ) / tunnel.size ) );
        return glm::clamp( coords, glm::ivec2( 0 ), grid_size - 1 );
    };
    auto const to_index = [ & ]( glm::ivec2 const& coords ) {
        return ( static_cast< size_t >( coords.y ) * static_cast< size_t >( grid_size.x ) )
             + static_cast< size_t >( coords.x );
    };

    auto const cell_count  = static_cast< size_t >( grid_size.x ) * static_cast< size_t >( grid_size.y );
    auto       cell_starts = std::vector< uint32 >( cell_count + 1UZ, 0U );
    auto       valid       = std::vector< uint32 >{ };
    valid.reserve( particles.size( ) );

    for ( auto index = 0U; index < particles.size( ); ++index )
    {
        auto const& particle = particles[ index ];

        if ( !std::isfinite( particle.position.x ) || !std::isfinite( particle.position.y )
             || !std::isfinite( particle.velocity.x ) || !std::isfinite( particle.velocity.y ) )
        {
            ++stats.invalid_count;
            continue;
        }

        if ( glm::any( glm::lessThan( particle.position, glm::vec2( 0.0F ) ) )
             || glm::any( glm::greaterThan( particle.position, tunnel.size ) ) )
        {
            ++stats.out_of_bounds_count;
        }

        auto const speed_sq = glm::dot( particle.velocity, particle.velocity );
        stats.kinetic_energy += 0.5 * static_cast< float64 >( speed_sq );
        stats.momentum += particle.velocity;
        stats.max_speed = std::max( stats.max_speed, std::sqrt( speed_sq ) );

        valid.push_back( index );
        ++cell_starts[ to_index( to_cell( particle.position ) ) + 1UZ ];
    }

    for ( auto cell = 0UZ; cell < cell_count; ++cell )
    {
        cell_starts[ cell + 1UZ ] += cell_starts[ cell ];
    }

    auto sorted = std::vector< uint32 >( valid.size( ) );
    auto next   = cell_starts;
    for ( auto const index : valid )
    {
        sorted[ next[ to_index( to_cell( particles[ index ].position ) ) ]++ ] = index;
    }

    for ( auto const index : valid )
    {
        auto const& self   = particles[ index ];
        auto const  coords = to_cell( self.position );
        auto const  lower  = glm::max( coords - 1, glm::ivec2( 0 ) );
        auto const  upper  = glm::min( coords + 1, grid_size - 1 );

        for ( auto y = lower.y; y <= upper.y; ++y )
        {
            auto const first = cell_starts[ to_index( { lower.x, y } ) ];
            auto const last  = cell_starts[ to_index( { upper.x, y } ) + 1UZ ];

            for ( auto slot = first; slot < last; ++slot )
            {
                auto const other = sorted[ slot ];
                // Visit each pair once.
                if ( other <= index )
                {
                    continue;
                }

                auto const dist = glm::distance( self.position, particles[ other ].position );
                if ( dist < diameter )
                {
                    ++stats.overlapping_pairs;
                    stats.max_overlap = std::max( stats.max_overlap, ( diameter - dist ) / diameter );
                }
            }
        }
    }

    return stats;
}

auto to_string( ParticleStats const& stats ) -> std::string
{
    return fmt::format(
        "count={} invalid={} out_of_bounds={} kinetic_energy={:.4f} momentum=({:.3f}, {:.3f}) "
        "max_speed={:.3f} overlapping_pairs={} max_overlap={:.3f}",
        stats.count,
        stats.invalid_count,
        stats.out_of_bounds_count,
        stats.kinetic_energy,
        stats.momentum.x,
        stats.momentum.y,
        stats.max_speed,
        stats.overlapping_pairs,
        stats.max_overlap
    );
}

auto check_stats( ParticleStats const& initial, ParticleStats const& current, bool const driven )
    -> utils::Result< void >
{
    auto const max_energy_ratio = driven ? 4.0 : 1.05;
    auto const min_energy_ratio = driven ? 0.25 : 0.75;

    LTB_CHECK_VALID( current.count == initial.count );
    LTB_CHECK_VALID( 0UZ == current.invalid_count );
    LTB_CHECK_VALID( 0UZ == current.out_of_bounds_count );
    // Some kinetic energy is stored in the springs of overlapping
    // particles but the total should not grow or drain away.
    LTB_CHECK_VALID( current.kinetic_energy <= initial.kinetic_energy * max_energy_ratio );
    LTB_CHECK_VALID( current.kinetic_energy >= initial.kinetic_energy * min_energy_ratio );
    // Particles should never pass through each other.
    LTB_CHECK_VALID( current.max_overlap < 0.9F );

    return utils::success( );
}

} // namespace ltb::flew
