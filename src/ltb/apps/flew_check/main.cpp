// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////

// Runs the simulation without a window and verifies the particles are still healthy.

// project
#include "ltb/flew/particle_system.hpp"
#include "ltb/flew/scenarios.hpp"
#include "ltb/utils/timers.hpp"
#include "ltb/vlk/check.hpp"
#include "ltb/vlk/command_buffer.hpp"

// external
#include <cxxopts.hpp>
#include <spdlog/spdlog.h>

// standard
#include <chrono>

namespace ltb::flew
{
namespace
{

struct CheckOptions
{
    ParticleSystemSettings system            = { };
    Scenario               scenario          = Scenario::Box;
    ScenarioSettings       scenario_settings = { };
    uint64                 steps             = 5'000UZ;
};

/// \brief The number of steps recorded into a single command buffer.
constexpr auto steps_per_submit = 50UZ;

auto run_check( CheckOptions options ) -> utils::Result< void >
{
    auto setup = make_scenario( options.scenario, options.system.tunnel, options.scenario_settings );

    options.system.solids             = std::move( setup.solids );
    options.system.fill.mean_velocity = { setup.flow_speed, 0.0F };

    auto gpu = vlk::objs::VulkanGpu{ };
    LTB_CHECK( gpu.initialize( { } ) );

    LTB_CHECK_VALID( gpu.device( ).queues( ).contains( vlk::QueueType::Compute ) );
    auto const queue = gpu.device( ).queues( ).at( vlk::QueueType::Compute );

    auto command_pool = vlk::CommandPool{ gpu.device( ), gpu.physical_device( ) };
    LTB_CHECK( command_pool.initialize( { .queue_type = vlk::QueueType::Compute } ) );

    auto particles = ParticleSystem{ gpu };
    LTB_CHECK( particles.initialize( options.system, command_pool, queue ) );

    auto params = default_step_params(
        particles.radius( ),
        options.system.fill.thermal_speed + setup.flow_speed
    );
    params.flow_speed    = setup.flow_speed;
    params.thermal_speed = options.system.fill.thermal_speed;

    LTB_CHECK( auto const initial_particles, particles.read_particles( command_pool, queue ) );
    auto const initial_stats
        = compute_stats( initial_particles, particles.tunnel( ), particles.radius( ) );

    auto command_buffer = vlk::CommandBuffer{ gpu.device( ), command_pool };
    LTB_CHECK( command_buffer.initialize( ) );

    auto const start_time = std::chrono::steady_clock::now( );

    for ( auto step = 0UZ; step < options.steps; step += steps_per_submit )
    {
        auto const step_count = std::min( steps_per_submit, options.steps - step );

        VK_CHECK( command_buffer.get( ).reset( ) );
        VK_CHECK( command_buffer.get( ).begin(
            vk::CommandBufferBeginInfo{ }.setFlags( vk::CommandBufferUsageFlagBits::eOneTimeSubmit )
        ) );
        for ( auto i = 0UZ; i < step_count; ++i )
        {
            particles.record_step( command_buffer.get( ), params );
        }
        VK_CHECK( command_buffer.get( ).end( ) );

        VK_CHECK( queue.submit( vk::SubmitInfo{ }.setCommandBuffers( command_buffer.get( ) ) ) );
        VK_CHECK( queue.waitIdle( ) );
    }

    auto const seconds
        = std::chrono::duration< float64 >( std::chrono::steady_clock::now( ) - start_time ).count( );

    LTB_CHECK( auto const final_particles, particles.read_particles( command_pool, queue ) );
    auto const final_stats
        = compute_stats( final_particles, particles.tunnel( ), particles.radius( ) );

    spdlog::info(
        "{}: {} steps of {} particles in {:.2f}s ({:.3f} ms/step)",
        scenario_names[ static_cast< size_t >( options.scenario ) ],
        options.steps,
        particles.particle_count( ),
        seconds,
        1000.0 * seconds / static_cast< float64 >( std::max( 1UZ, options.steps ) )
    );
    spdlog::info( "Initial: {}", to_string( initial_stats ) );
    spdlog::info( "Final  : {}", to_string( final_stats ) );

    // No particle may end up inside a solid.
    auto inside_count = 0UZ;
    for ( auto const& particle : final_particles )
    {
        if ( signed_distance( particles.settings( ).solids, particle.position ) < -particles.radius( ) )
        {
            ++inside_count;
        }
    }
    spdlog::info( "Particles inside solids: {}", inside_count );
    LTB_CHECK_VALID( 0UZ == inside_count );

    return check_stats( initial_stats, final_stats, setup.flow_speed > 0.0F );
}

} // namespace
} // namespace ltb::flew

auto main( ltb::int32 const argc, char const* argv[] ) -> int
{
    auto options = cxxopts::Options{ "flew-check", "Headless particle simulation check." };
    options.add_options( )
        ( "p,particles", "Number of particles", cxxopts::value< ltb::uint32 >( ) )
        ( "scenario", "Box, Cylinder, Airfoil, Plate, or Venturi", cxxopts::value< std::string >( ) )
        ( "s,steps", "Number of simulation steps", cxxopts::value< ltb::uint64 >( ) )
        ( "h,help", "Print usage" );

    auto check_options = ltb::flew::CheckOptions{ };

    try
    {
        auto const args = options.parse( argc, argv );

        if ( args.count( "help" ) > 0UZ )
        {
            std::puts( options.help( ).c_str( ) );
            return EXIT_SUCCESS;
        }
        if ( args.count( "particles" ) > 0UZ )
        {
            check_options.system.fill.particle_count = args[ "particles" ].as< ltb::uint32 >( );
        }
        if ( args.count( "scenario" ) > 0UZ )
        {
            auto const name     = args[ "scenario" ].as< std::string >( );
            auto const scenario = ltb::flew::scenario_from_name( name );
            if ( !scenario )
            {
                spdlog::error( "Unknown scenario: {}", name );
                return EXIT_FAILURE;
            }
            check_options.scenario = scenario.value( );
        }
        if ( args.count( "steps" ) > 0UZ )
        {
            check_options.steps = args[ "steps" ].as< ltb::uint64 >( );
        }
    }
    catch ( cxxopts::OptionException const& error )
    {
        spdlog::error( "{}", error.what( ) );
        return EXIT_FAILURE;
    }

    if ( auto result = ltb::flew::run_check( check_options ); !result )
    {
        spdlog::error( "Check failed:\n{}", result.error( ).debug_error_message( ) );
        return EXIT_FAILURE;
    }

    spdlog::info( "Check passed" );
    return EXIT_SUCCESS;
}
