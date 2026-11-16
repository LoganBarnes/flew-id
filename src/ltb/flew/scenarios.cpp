// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "ltb/flew/scenarios.hpp"

// external
#include <glm/gtc/constants.hpp>

namespace ltb::flew
{

auto scenario_from_name( std::string_view const name ) -> std::optional< Scenario >
{
    for ( auto index = 0U; index < scenario_names.size( ); ++index )
    {
        if ( name == scenario_names[ index ] )
        {
            return static_cast< Scenario >( index );
        }
    }
    return std::nullopt;
}

auto scenario_tunnel( Scenario const scenario ) -> Tunnel
{
    if ( Scenario::LongVenturi == scenario )
    {
        return Tunnel{ .size = { 8.0F, 2.0F } };
    }
    return Tunnel{ };
}

auto default_scenario_settings( Scenario const scenario ) -> ScenarioSettings
{
    auto settings = ScenarioSettings{ };

    if ( Scenario::Venturi == scenario || Scenario::LongVenturi == scenario )
    {
        settings.flow_speed      = 0.2F;
        settings.throat_fraction = 0.5F;
    }
    return settings;
}

auto venturi_walls(
    Tunnel const& tunnel,
    float32 const start_x,
    float32 const ramp_length,
    float32 const throat_length,
    float32 const throat_height
) -> std::vector< Polygon >
{
    constexpr auto ramp_points = 32U;

    auto const& size        = tunnel.size;
    auto const  wall_height = 0.5F * ( size.y - throat_height );
    auto const  end_x       = start_x + ( 2.0F * ramp_length ) + throat_length;

    // The height of the bottom wall along the tunnel.
    auto profile = Polygon{ };
    for ( auto i = 0U; i <= ramp_points; ++i )
    {
        auto const t = static_cast< float32 >( i ) / static_cast< float32 >( ramp_points );
        // Eases from zero to the full height with no corners.
        auto const h = wall_height * 0.5F * ( 1.0F - std::cos( t * glm::pi< float32 >( ) ) );

        profile.push_back( { start_x + ( t * ramp_length ), h } );
    }
    for ( auto i = ramp_points + 1U; i-- > 0U; )
    {
        auto const& up = profile[ i ];
        // The widening ramp mirrors the narrowing one.
        profile.push_back( { end_x - ( up.x - start_x ), up.y } );
    }

    auto bottom = profile;
    auto top    = Polygon{ };
    for ( auto const& point : profile )
    {
        top.push_back( { point.x, size.y - point.y } );
    }

    // Close the shapes outside of the tunnel.
    bottom.push_back( { end_x, -size.y } );
    bottom.push_back( { start_x, -size.y } );
    top.push_back( { end_x, 2.0F * size.y } );
    top.push_back( { start_x, 2.0F * size.y } );

    return { std::move( bottom ), std::move( top ) };
}

auto make_scenario(
    Scenario const          scenario,
    Tunnel const&           tunnel,
    ScenarioSettings const& settings
) -> ScenarioSetup
{
    auto const& size  = tunnel.size;
    auto const  angle = glm::radians( settings.angle_degrees );

    auto setup = ScenarioSetup{ .flow_speed = settings.flow_speed };

    switch ( scenario )
    {
        using enum Scenario;

        case Box:
            setup.flow_speed = 0.0F;
            break;

        case Cylinder:
            setup.solids.circles.push_back( {
                .center = { size.x * 0.3F, size.y * 0.5F },
                .radius = size.y * 0.1F,
            } );
            break;

        case Airfoil:
        {
            auto const chord = size.x * 0.25F;
            // Positive angles of attack raise the leading edge (a clockwise rotation).
            setup.solids.polygons.push_back(
                transformed(
                    naca_airfoil( 4412U, chord ),
                    -angle,
                    { chord * 0.25F, 0.0F },
                    { size.x * 0.3F, size.y * 0.5F }
                )
            );
        }
        break;

        case Plate:
        {
            auto const half_length    = size.x * 0.1F;
            auto const half_thickness = size.y * 0.01F;
            setup.solids.polygons.push_back(
                transformed(
                    rectangle( { -half_length, -half_thickness }, { half_length, half_thickness } ),
                    -angle,
                    { 0.0F, 0.0F },
                    { size.x * 0.35F, size.y * 0.5F }
                )
            );
        }
        break;

        case Venturi:
            // A short constriction with no straight section.
            setup.solids.polygons = venturi_walls(
                tunnel,
                size.x * 0.3F,
                size.x * 0.2F,
                0.0F,
                size.y * settings.throat_fraction
            );
            break;

        case LongVenturi:
            // Gentle slopes and a long narrow section, so the flow has room to settle
            // at its new speed and pressure before the tunnel widens again.
            setup.solids.polygons = venturi_walls(
                tunnel,
                size.x * 0.075F,
                size.x * 0.3F,
                size.x * 0.25F,
                size.y * settings.throat_fraction
            );
            break;
    }

    return setup;
}

} // namespace ltb::flew
