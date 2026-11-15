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
        {
            // Two smooth bumps narrowing the middle of the tunnel to a third of its height.
            constexpr auto point_count = 64U;
            auto const     start       = size.x * 0.3F;
            auto const     length      = size.x * 0.4F;
            auto const     bump_height = size.y / 3.0F;

            auto bottom = Polygon{ };
            auto top    = Polygon{ };
            for ( auto i = 0U; i <= point_count; ++i )
            {
                auto const t = static_cast< float32 >( i ) / static_cast< float32 >( point_count );
                auto const x = start + ( t * length );
                auto const h = bump_height * 0.5F * ( 1.0F - std::cos( t * glm::two_pi< float32 >( ) ) );

                bottom.push_back( { x, h } );
                top.push_back( { x, size.y - h } );
            }
            // Close the shapes outside of the tunnel.
            bottom.push_back( { start + length, -size.y } );
            bottom.push_back( { start, -size.y } );
            top.push_back( { start + length, 2.0F * size.y } );
            top.push_back( { start, 2.0F * size.y } );

            setup.solids.polygons.push_back( std::move( bottom ) );
            setup.solids.polygons.push_back( std::move( top ) );
        }
        break;
    }

    return setup;
}

} // namespace ltb::flew
