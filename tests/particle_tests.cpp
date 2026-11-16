// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "ltb/flew/particle.hpp"

// external
#include <glm/gtc/constants.hpp>
#include <gtest/gtest.h>

// standard
#include <cmath>
#include <limits>

namespace ltb::flew
{
namespace
{

auto same_particles( std::vector< Particle > const& lhs, std::vector< Particle > const& rhs ) -> bool
{
    return std::ranges::equal( lhs, rhs, []( Particle const& a, Particle const& b ) {
        return a.position == b.position && a.velocity == b.velocity;
    } );
}

TEST( ParticleRadiusTests, ParticlesCoverThePackingFractionOfTheTunnel )
{
    auto const tunnel   = Tunnel{ .size = { 4.0F, 2.0F } };
    auto const settings = ParticleFillSettings{ .particle_count = 1'000U, .packing_fraction = 0.4F };

    auto const radius        = particle_radius( tunnel, settings );
    auto const particle_area = 1'000.0F * glm::pi< float32 >( ) * radius * radius;

    EXPECT_NEAR( particle_area, 0.4F * 8.0F, 1.0e-4F );
}

TEST( ParticleRadiusTests, MoreParticlesAreSmaller )
{
    auto const tunnel = Tunnel{ };

    auto const few  = particle_radius( tunnel, { .particle_count = 1'000U } );
    auto const many = particle_radius( tunnel, { .particle_count = 4'000U } );

    // Four times as many particles covering the same area have half the radius.
    EXPECT_NEAR( many, few * 0.5F, 1.0e-6F );
}

class FillTunnelTests : public ::testing::Test
{
protected:
    Tunnel               tunnel   = { .size = { 4.0F, 2.0F } };
    ParticleFillSettings settings = {
        .particle_count   = 20'000U,
        .packing_fraction = 0.4F,
        .thermal_speed    = 0.5F,
        .mean_velocity    = { 0.3F, -0.1F },
        .seed             = 7U,
    };
    float32 radius = particle_radius( tunnel, settings );
};

TEST_F( FillTunnelTests, CreatesTheRequestedNumberOfParticles )
{
    EXPECT_EQ( fill_tunnel( tunnel, settings ).size( ), 20'000UZ );

    settings.particle_count = 0U;
    EXPECT_TRUE( fill_tunnel( tunnel, settings ).empty( ) );

    settings.particle_count = 1U;
    EXPECT_EQ( fill_tunnel( tunnel, settings ).size( ), 1UZ );
}

TEST_F( FillTunnelTests, ParticlesAreInsideTheTunnelAndDoNotOverlap )
{
    auto const particles = fill_tunnel( tunnel, settings );
    auto const stats     = compute_stats( particles, tunnel, radius );

    EXPECT_EQ( stats.invalid_count, 0UZ );
    EXPECT_EQ( stats.out_of_bounds_count, 0UZ );
    EXPECT_EQ( stats.overlapping_pairs, 0UZ );

    // The whole particle is inside, not just its center.
    for ( auto const& particle : particles )
    {
        ASSERT_GE( particle.position.x, radius - 1.0e-6F );
        ASSERT_GE( particle.position.y, radius - 1.0e-6F );
        ASSERT_LE( particle.position.x, tunnel.size.x - radius + 1.0e-6F );
        ASSERT_LE( particle.position.y, tunnel.size.y - radius + 1.0e-6F );
    }
}

TEST_F( FillTunnelTests, ParticlesAreSpreadOverTheWholeTunnel )
{
    auto const particles = fill_tunnel( tunnel, settings );

    // Count the particles in each quarter of the tunnel.
    auto counts = std::array< uint32, 4 >{ };
    for ( auto const& particle : particles )
    {
        auto const right = ( particle.position.x > tunnel.size.x * 0.5F ) ? 1UZ : 0UZ;
        auto const upper = ( particle.position.y > tunnel.size.y * 0.5F ) ? 2UZ : 0UZ;
        ++counts[ right + upper ];
    }
    for ( auto const count : counts )
    {
        EXPECT_NEAR( static_cast< float32 >( count ), 5'000.0F, 250.0F );
    }
}

TEST_F( FillTunnelTests, VelocitiesHaveTheRequestedMeanAndThermalSpeed )
{
    auto const particles = fill_tunnel( tunnel, settings );

    auto mean = glm::dvec2{ 0.0 };
    for ( auto const& particle : particles )
    {
        mean += glm::dvec2( particle.velocity );
    }
    mean /= static_cast< float64 >( particles.size( ) );

    EXPECT_NEAR( mean.x, 0.3, 0.02 );
    EXPECT_NEAR( mean.y, -0.1, 0.02 );

    // The thermal speed is the root mean square speed relative to the mean velocity.
    auto speed_sq = 0.0;
    for ( auto const& particle : particles )
    {
        auto const thermal = glm::dvec2( particle.velocity ) - mean;
        speed_sq += glm::dot( thermal, thermal );
    }
    auto const rms = std::sqrt( speed_sq / static_cast< float64 >( particles.size( ) ) );

    EXPECT_NEAR( rms, 0.5, 0.02 );
}

TEST_F( FillTunnelTests, ParticlesWithNoThermalSpeedAllMoveAtTheMeanVelocity )
{
    settings.thermal_speed = 0.0F;

    for ( auto const& particle : fill_tunnel( tunnel, settings ) )
    {
        ASSERT_EQ( particle.velocity, settings.mean_velocity );
    }
}

TEST_F( FillTunnelTests, IsDeterministicForASeed )
{
    auto const first = fill_tunnel( tunnel, settings );
    EXPECT_TRUE( same_particles( first, fill_tunnel( tunnel, settings ) ) );

    settings.seed = 8U;
    EXPECT_FALSE( same_particles( first, fill_tunnel( tunnel, settings ) ) );
}

TEST_F( FillTunnelTests, ParticlesAvoidSolids )
{
    // A circle covering a large part of the tunnel.
    auto const center       = glm::vec2{ 1.5F, 1.0F };
    auto const solid_radius = 0.7F;
    auto const distance     = [ & ]( glm::vec2 const& point ) {
        return glm::distance( point, center ) - solid_radius;
    };

    auto const particles = fill_tunnel( tunnel, settings, distance );
    auto const stats     = compute_stats( particles, tunnel, radius );

    ASSERT_EQ( particles.size( ), 20'000UZ );
    EXPECT_EQ( stats.out_of_bounds_count, 0UZ );
    EXPECT_EQ( stats.overlapping_pairs, 0UZ );

    for ( auto const& particle : particles )
    {
        // Not even touching the solid.
        ASSERT_GT( distance( particle.position ), radius );
    }
}

class ComputeStatsTests : public ::testing::Test
{
protected:
    Tunnel  tunnel = { .size = { 4.0F, 2.0F } };
    float32 radius = 0.1F;
};

TEST_F( ComputeStatsTests, EmptyInput )
{
    auto const stats = compute_stats( { }, tunnel, radius );

    EXPECT_EQ( stats.count, 0UZ );
    EXPECT_EQ( stats.overlapping_pairs, 0UZ );
    EXPECT_DOUBLE_EQ( stats.kinetic_energy, 0.0 );
}

TEST_F( ComputeStatsTests, EnergyMomentumAndSpeed )
{
    auto const particles = std::vector< Particle >{
        { .position = { 1.0F, 1.0F }, .velocity = { 1.0F, 0.0F } },
        { .position = { 3.0F, 1.0F }, .velocity = { 0.0F, 2.0F } },
        { .position = { 2.0F, 0.5F }, .velocity = { -3.0F, 4.0F } },
    };
    auto const stats = compute_stats( particles, tunnel, radius );

    EXPECT_EQ( stats.count, 3UZ );
    EXPECT_EQ( stats.invalid_count, 0UZ );
    EXPECT_EQ( stats.out_of_bounds_count, 0UZ );
    // 0.5 * (1 + 4 + 25)
    EXPECT_NEAR( stats.kinetic_energy, 15.0, 1.0e-6 );
    EXPECT_NEAR( stats.momentum.x, -2.0F, 1.0e-6F );
    EXPECT_NEAR( stats.momentum.y, 6.0F, 1.0e-6F );
    EXPECT_NEAR( stats.max_speed, 5.0F, 1.0e-6F );
    EXPECT_EQ( stats.overlapping_pairs, 0UZ );
    EXPECT_FLOAT_EQ( stats.max_overlap, 0.0F );
}

TEST_F( ComputeStatsTests, OverlappingPairs )
{
    // The diameter is 0.2.
    auto const particles = std::vector< Particle >{
        // Half overlapping.
        { .position = { 1.0F, 1.0F } },
        { .position = { 1.1F, 1.0F } },
        // A quarter overlapping (diagonally: 0.15 apart).
        { .position = { 3.0F, 1.5F } },
        { .position = { 3.09F, 1.62F } },
        // Just touching, and alone.
        { .position = { 2.0F, 0.5F } },
        { .position = { 2.2001F, 0.5F } },
        { .position = { 0.5F, 0.5F } },
    };
    auto const stats = compute_stats( particles, tunnel, radius );

    EXPECT_EQ( stats.overlapping_pairs, 2UZ );
    EXPECT_NEAR( stats.max_overlap, 0.5F, 1.0e-4F );
}

TEST_F( ComputeStatsTests, FindsOverlapsNoMatterWhereThePairIs )
{
    // Slide an overlapping pair across the tunnel in steps that don't line up with the
    // cells used to find neighbors, so the pair straddles cell boundaries in both directions.
    for ( auto step = 0; step < 150; ++step )
    {
        auto const offset    = static_cast< float32 >( step ) * 0.0123F;
        auto const first     = glm::vec2{ 0.3F + offset, 0.2F + ( offset * 0.8F ) };
        auto const particles = std::vector< Particle >{
            { .position = first },
            { .position = first + glm::vec2{ 0.06F, 0.08F } },
        };
        auto const stats = compute_stats( particles, tunnel, radius );

        ASSERT_EQ( stats.overlapping_pairs, 1UZ ) << "Step " << step;
        ASSERT_NEAR( stats.max_overlap, 0.5F, 1.0e-3F ) << "Step " << step;
    }
}

TEST_F( ComputeStatsTests, InvalidAndOutOfBoundsParticles )
{
    constexpr auto nan = std::numeric_limits< float32 >::quiet_NaN( );
    constexpr auto inf = std::numeric_limits< float32 >::infinity( );

    auto const particles = std::vector< Particle >{
        { .position = { 1.0F, 1.0F }, .velocity = { 1.0F, 0.0F } },
        { .position = { nan, 1.0F } },
        { .position = { 1.0F, 1.0F }, .velocity = { 0.0F, inf } },
        { .position = { -0.5F, 1.0F } },
        { .position = { 1.0F, 2.5F } },
    };
    auto const stats = compute_stats( particles, tunnel, radius );

    EXPECT_EQ( stats.count, 5UZ );
    EXPECT_EQ( stats.invalid_count, 2UZ );
    EXPECT_EQ( stats.out_of_bounds_count, 2UZ );
    // Invalid particles are left out of the totals.
    EXPECT_NEAR( stats.kinetic_energy, 0.5, 1.0e-6 );
}

class CheckStatsTests : public ::testing::Test
{
protected:
    ParticleStats initial = { .count = 100UZ, .kinetic_energy = 50.0 };
};

TEST_F( CheckStatsTests, UnchangedStatsPass )
{
    EXPECT_TRUE( check_stats( initial, initial ).has_value( ) );
}

TEST_F( CheckStatsTests, LostParticlesFail )
{
    auto current  = initial;
    current.count = 99UZ;
    EXPECT_FALSE( check_stats( initial, current ).has_value( ) );
}

TEST_F( CheckStatsTests, InvalidOrEscapedParticlesFail )
{
    auto current          = initial;
    current.invalid_count = 1UZ;
    EXPECT_FALSE( check_stats( initial, current ).has_value( ) );

    current                     = initial;
    current.out_of_bounds_count = 1UZ;
    EXPECT_FALSE( check_stats( initial, current ).has_value( ) );
}

TEST_F( CheckStatsTests, ParticlesPassingThroughEachOtherFail )
{
    auto current        = initial;
    current.max_overlap = 0.5F;
    EXPECT_TRUE( check_stats( initial, current ).has_value( ) );

    current.max_overlap = 0.95F;
    EXPECT_FALSE( check_stats( initial, current ).has_value( ) );
}

TEST_F( CheckStatsTests, EnergyMustBeConservedUnlessDriven )
{
    auto current = initial;

    // A little energy is stored in the springs between touching particles.
    current.kinetic_energy = 45.0;
    EXPECT_TRUE( check_stats( initial, current ).has_value( ) );

    current.kinetic_energy = 30.0;
    EXPECT_FALSE( check_stats( initial, current ).has_value( ) );
    EXPECT_TRUE( check_stats( initial, current, true ).has_value( ) );

    current.kinetic_energy = 60.0;
    EXPECT_FALSE( check_stats( initial, current ).has_value( ) );
    EXPECT_TRUE( check_stats( initial, current, true ).has_value( ) );

    // Even a driven simulation shouldn't blow up or die out.
    current.kinetic_energy = 500.0;
    EXPECT_FALSE( check_stats( initial, current, true ).has_value( ) );
    current.kinetic_energy = 5.0;
    EXPECT_FALSE( check_stats( initial, current, true ).has_value( ) );
}

TEST_F( CheckStatsTests, ADrivenFluidMayStartAtRest )
{
    initial.kinetic_energy = 0.0;

    // A little motion is fine (an allowance of 0.01 per particle).
    auto current           = initial;
    current.kinetic_energy = 0.5;
    EXPECT_TRUE( check_stats( initial, current, true ).has_value( ) );
    EXPECT_FALSE( check_stats( initial, current ).has_value( ) );

    current.kinetic_energy = 5.0;
    EXPECT_FALSE( check_stats( initial, current, true ).has_value( ) );
}

TEST( ParticleStatsTests, ToStringIncludesTheValues )
{
    auto const text = to_string( ParticleStats{ .count = 123UZ, .overlapping_pairs = 45UZ } );

    EXPECT_NE( text.find( "count=123" ), std::string::npos );
    EXPECT_NE( text.find( "overlapping_pairs=45" ), std::string::npos );
}

} // namespace
} // namespace ltb::flew
