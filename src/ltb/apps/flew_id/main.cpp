// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////

// project
#include "app.hpp"
#include "ltb/exec/app_main.hpp"

// external
#include <cxxopts.hpp>

auto main( ltb::int32 const argc, char const* argv[] ) -> int
{
    auto options = cxxopts::Options{ "flew-id", "A particle wind tunnel." };
    options.add_options( )
        ( "p,particles", "Number of particles", cxxopts::value< ltb::uint32 >( ) )
        ( "scenario", "Box, Cylinder, Airfoil, Plate, Venturi, or LongVenturi", cxxopts::value< std::string >( ) )
        ( "angle", "Angle of attack in degrees (Airfoil, Plate)", cxxopts::value< ltb::float32 >( ) )
        ( "flow", "Flow speed", cxxopts::value< ltb::float32 >( ) )
        ( "throat", "Open fraction of the tunnel height at the venturi's narrowest point",
          cxxopts::value< ltb::float32 >( ) )
        ( "field", "Background: None, Speed, Pressure, Density, Vorticity, or Temperature",
          cxxopts::value< std::string >( ) )
        ( "respawn", "Replace particles leaving the ends of the tunnel with new incoming particles" )
        ( "hide-particles", "Only draw the background" )
        ( "s,steps", "Exit after this many simulation steps", cxxopts::value< ltb::uint64 >( ) )
        ( "video", "Record every frame to this video file (requires ffmpeg)",
          cxxopts::value< std::string >( ) )
        ( "fps", "Frame rate of the recorded video", cxxopts::value< ltb::uint32 >( ) )
        ( "steps-per-frame", "Simulation steps per update (and per recorded frame)",
          cxxopts::value< ltb::uint32 >( ) )
        ( "hide-gui", "Don't draw the settings window" )
        ( "o,screenshot", "Write the last frame to this PNG file (requires --steps)",
          cxxopts::value< std::string >( ) )
        ( "c,check", "Verify the particles after --steps (exit code is non-zero on failure)" )
        ( "h,help", "Print usage" );

    try
    {
        auto const args = options.parse( argc, argv );

        if ( args.count( "help" ) > 0UZ )
        {
            std::puts( options.help( ).c_str( ) );
            return EXIT_SUCCESS;
        }

        auto& app_options = ltb::flew::app_options( );
        if ( args.count( "particles" ) > 0UZ )
        {
            app_options.particle_count = args[ "particles" ].as< ltb::uint32 >( );
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
            app_options.scenario = scenario.value( );
        }
        if ( args.count( "angle" ) > 0UZ )
        {
            app_options.scenario_settings.angle_degrees = args[ "angle" ].as< ltb::float32 >( );
        }
        if ( args.count( "throat" ) > 0UZ )
        {
            app_options.scenario_settings.throat_fraction = args[ "throat" ].as< ltb::float32 >( );
        }
        if ( args.count( "flow" ) > 0UZ )
        {
            app_options.scenario_settings.flow_speed = args[ "flow" ].as< ltb::float32 >( );
        }
        if ( args.count( "field" ) > 0UZ )
        {
            app_options.field = args[ "field" ].as< std::string >( );
        }
        app_options.respawn        = ( args.count( "respawn" ) > 0UZ );
        app_options.hide_particles = ( args.count( "hide-particles" ) > 0UZ );
        if ( args.count( "steps" ) > 0UZ )
        {
            app_options.max_steps = args[ "steps" ].as< ltb::uint64 >( );
        }
        if ( args.count( "video" ) > 0UZ )
        {
            app_options.video = args[ "video" ].as< std::string >( );
        }
        if ( args.count( "fps" ) > 0UZ )
        {
            app_options.video_fps = args[ "fps" ].as< ltb::uint32 >( );
        }
        if ( args.count( "steps-per-frame" ) > 0UZ )
        {
            app_options.steps_per_update = args[ "steps-per-frame" ].as< ltb::uint32 >( );
        }
        app_options.hide_gui = ( args.count( "hide-gui" ) > 0UZ );
        if ( args.count( "screenshot" ) > 0UZ )
        {
            app_options.screenshot = args[ "screenshot" ].as< std::string >( );
        }
        app_options.check = ( args.count( "check" ) > 0UZ );
    }
    catch ( cxxopts::OptionException const& error )
    {
        spdlog::error( "{}", error.what( ) );
        return EXIT_FAILURE;
    }

    auto const exit_code = ltb::exec::windowed_app_main< ltb::flew::FlewIdApp >( argc, argv );

    return ltb::flew::app_options( ).failed ? EXIT_FAILURE : exit_code;
}
