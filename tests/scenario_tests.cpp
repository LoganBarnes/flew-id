// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "ltb/flew/scenarios.hpp"

// project
#include "ltb/flew/particle_system.hpp"

// external
#include <gtest/gtest.h>

// standard
#include <algorithm>
#include <limits>

namespace ltb::flew
{
namespace
{

constexpr auto all_scenarios = std::array{
    Scenario::Box,
    Scenario::Cylinder,
    Scenario::Airfoil,
    Scenario::Plate,
    Scenario::Venturi,
    Scenario::LongVenturi,
};

/// \brief The fraction of the tunnel's height that is open at `x`.
auto open_fraction( SolidScene const& scene, Tunnel const& tunnel, float32 const x ) -> float32
{
    constexpr auto samples = 1'000;

    auto open = 0;
    for ( auto i = 0; i < samples; ++i )
    {
        auto const y = ( static_cast< float32 >( i ) + 0.5F ) * tunnel.size.y
                     / static_cast< float32 >( samples );
        if ( signed_distance( scene, { x, y } ) > 0.0F )
        {
            ++open;
        }
    }
    return static_cast< float32 >( open ) / static_cast< float32 >( samples );
}

/// \brief The length of the flat, highest part of the bottom wall of a venturi.
auto throat_length( Polygon const& bottom_wall ) -> float32
{
    auto const by_y   = []( glm::vec2 const& a, glm::vec2 const& b ) { return a.y < b.y; };
    auto const height = std::ranges::max_element( bottom_wall, by_y )->y;

    auto min_x = std::numeric_limits< float32 >::max( );
    auto max_x = std::numeric_limits< float32 >::lowest( );
    for ( auto const& point : bottom_wall )
    {
        if ( point.y > height - 1.0e-5F )
        {
            min_x = std::min( min_x, point.x );
            max_x = std::max( max_x, point.x );
        }
    }
    return max_x - min_x;
}

/// \brief The steepest slope of the bottom wall of a venturi.
auto steepest_slope( Polygon const& bottom_wall ) -> float32
{
    auto steepest = 0.0F;
    for ( auto i = 1UZ; i < bottom_wall.size( ); ++i )
    {
        auto const delta = bottom_wall[ i ] - bottom_wall[ i - 1UZ ];
        // Only the part of the wall inside the tunnel.
        if ( bottom_wall[ i ].y >= 0.0F && bottom_wall[ i - 1UZ ].y >= 0.0F && delta.x > 1.0e-6F )
        {
            steepest = std::max( steepest, std::abs( delta.y / delta.x ) );
        }
    }
    return steepest;
}

/// \brief The fraction of the tunnel covered by solids (sampled on a grid).
auto solid_fraction( SolidScene const& scene, Tunnel const& tunnel ) -> float32
{
    constexpr auto columns = 200;
    constexpr auto rows    = 100;

    auto inside = 0;
    for ( auto y = 0; y < rows; ++y )
    {
        for ( auto x = 0; x < columns; ++x )
        {
            auto const point = ( glm::vec2( glm::ivec2{ x, y } ) + 0.5F ) * tunnel.size
                             / glm::vec2( glm::ivec2{ columns, rows } );
            if ( signed_distance( scene, point ) < 0.0F )
            {
                ++inside;
            }
        }
    }
    return static_cast< float32 >( inside ) / static_cast< float32 >( columns * rows );
}

TEST( ScenarioNameTests, EveryScenarioHasAName )
{
    ASSERT_EQ( scenario_names.size( ), all_scenarios.size( ) );

    for ( auto const scenario : all_scenarios )
    {
        auto const name = std::string_view{ scenario_names[ static_cast< size_t >( scenario ) ] };

        EXPECT_FALSE( name.empty( ) );
        EXPECT_EQ( scenario_from_name( name ), scenario ) << name;
    }
}

TEST( ScenarioNameTests, UnknownNamesAreNotScenarios )
{
    EXPECT_EQ( scenario_from_name( "" ), std::nullopt );
    EXPECT_EQ( scenario_from_name( "Teapot" ), std::nullopt );
    // Names are case sensitive.
    EXPECT_EQ( scenario_from_name( "airfoil" ), std::nullopt );
}

class ScenarioTests : public ::testing::Test
{
protected:
    Tunnel           tunnel   = { .size = { 4.0F, 2.0F } };
    ScenarioSettings settings = { .angle_degrees = 8.0F, .flow_speed = 0.7F };
};

TEST_F( ScenarioTests, BoxIsEmptyAndStill )
{
    auto const setup = make_scenario( Scenario::Box, tunnel, settings );

    EXPECT_TRUE( setup.solids.polygons.empty( ) );
    EXPECT_TRUE( setup.solids.circles.empty( ) );
    EXPECT_FLOAT_EQ( setup.flow_speed, 0.0F );
}

TEST_F( ScenarioTests, FlowScenariosHaveSolidsThatLeaveRoomForTheFluid )
{
    for ( auto const scenario : all_scenarios )
    {
        if ( Scenario::Box == scenario )
        {
            continue;
        }
        auto const* const name = scenario_names[ static_cast< size_t >( scenario ) ];

        tunnel           = scenario_tunnel( scenario );
        auto const setup = make_scenario( scenario, tunnel, settings );

        EXPECT_FLOAT_EQ( setup.flow_speed, 0.7F ) << name;

        auto const fraction = solid_fraction( setup.solids, tunnel );
        EXPECT_GT( fraction, 0.001F ) << name;
        EXPECT_LT( fraction, 0.5F ) << name;

        // The buffer zones at both ends of the tunnel must be clear of solids.
        auto const buffer_width = StepParams{ }.buffer_width;
        for ( auto y = 0.05F; y < tunnel.size.y; y += 0.1F )
        {
            EXPECT_GT( signed_distance( setup.solids, { buffer_width, y } ), 0.05F ) << name;
            EXPECT_GT( signed_distance( setup.solids, { tunnel.size.x - buffer_width, y } ), 0.05F )
                << name;
        }
    }
}

TEST_F( ScenarioTests, ObstaclesAreFullyInsideTheTunnel )
{
    for ( auto const scenario : { Scenario::Cylinder, Scenario::Airfoil, Scenario::Plate } )
    {
        auto const* const name  = scenario_names[ static_cast< size_t >( scenario ) ];
        auto const        setup = make_scenario( scenario, tunnel, settings );

        for ( auto const& polygon : setup.solids.polygons )
        {
            for ( auto const& point : polygon )
            {
                EXPECT_GT( point.x, 0.0F ) << name;
                EXPECT_GT( point.y, 0.0F ) << name;
                EXPECT_LT( point.x, tunnel.size.x ) << name;
                EXPECT_LT( point.y, tunnel.size.y ) << name;
            }
        }
        for ( auto const& circle : setup.solids.circles )
        {
            EXPECT_GT( circle.center.x - circle.radius, 0.0F ) << name;
            EXPECT_GT( circle.center.y - circle.radius, 0.0F ) << name;
            EXPECT_LT( circle.center.x + circle.radius, tunnel.size.x ) << name;
            EXPECT_LT( circle.center.y + circle.radius, tunnel.size.y ) << name;
        }
    }
}

TEST_F( ScenarioTests, PositiveAnglesOfAttackRaiseTheLeadingEdge )
{
    for ( auto const scenario : { Scenario::Airfoil, Scenario::Plate } )
    {
        auto const* const name = scenario_names[ static_cast< size_t >( scenario ) ];

        // The height of the leading (leftmost) edge relative to the trailing (rightmost) edge.
        auto const leading_edge_rise = [ & ]( float32 const angle_degrees ) {
            settings.angle_degrees = angle_degrees;
            auto const setup       = make_scenario( scenario, tunnel, settings );
            auto const& polygon    = setup.solids.polygons.front( );

            auto const by_x = []( glm::vec2 const& a, glm::vec2 const& b ) { return a.x < b.x; };
            auto const [ leading, trailing ] = std::ranges::minmax_element( polygon, by_x );
            return leading->y - trailing->y;
        };

        EXPECT_GT( leading_edge_rise( 20.0F ), 0.05F ) << name;
        EXPECT_LT( leading_edge_rise( -20.0F ), -0.05F ) << name;
        EXPECT_GT( leading_edge_rise( 20.0F ), leading_edge_rise( 5.0F ) ) << name;
    }
}

TEST_F( ScenarioTests, VenturiNarrowsTheMiddleOfTheTunnelToAThird )
{
    auto const setup    = make_scenario( Scenario::Venturi, tunnel, settings );
    auto const center_x = tunnel.size.x * 0.5F;

    EXPECT_NEAR( open_fraction( setup.solids, tunnel, center_x ), 1.0F / 3.0F, 0.01F );

    // The middle is open and the constriction is symmetric about it.
    EXPECT_GT( signed_distance( setup.solids, { center_x, tunnel.size.y * 0.5F } ), 0.0F );
    EXPECT_LT( signed_distance( setup.solids, { center_x, tunnel.size.y * 0.1F } ), 0.0F );
    EXPECT_LT( signed_distance( setup.solids, { center_x, tunnel.size.y * 0.9F } ), 0.0F );

    // Fully open before and after the constriction.
    EXPECT_FLOAT_EQ( open_fraction( setup.solids, tunnel, tunnel.size.x * 0.2F ), 1.0F );
    EXPECT_FLOAT_EQ( open_fraction( setup.solids, tunnel, tunnel.size.x * 0.8F ), 1.0F );
}

TEST( ScenarioDefaultTests, VenturiScenariosUseASlowerFlowAndWiderThroat )
{
    auto const general = default_scenario_settings( Scenario::Airfoil );

    for ( auto const scenario : { Scenario::Venturi, Scenario::LongVenturi } )
    {
        auto const settings = default_scenario_settings( scenario );

        EXPECT_FLOAT_EQ( settings.flow_speed, 0.2F );
        EXPECT_FLOAT_EQ( settings.throat_fraction, 0.5F );
        EXPECT_LT( settings.flow_speed, general.flow_speed );
        EXPECT_GT( settings.throat_fraction, general.throat_fraction );
    }
}

TEST( ScenarioDefaultTests, OtherScenariosUseTheGeneralDefaults )
{
    for ( auto const scenario : { Scenario::Box, Scenario::Cylinder, Scenario::Airfoil, Scenario::Plate } )
    {
        auto const settings = default_scenario_settings( scenario );

        EXPECT_FLOAT_EQ( settings.flow_speed, ScenarioSettings{ }.flow_speed );
        EXPECT_FLOAT_EQ( settings.angle_degrees, ScenarioSettings{ }.angle_degrees );
        EXPECT_FLOAT_EQ( settings.throat_fraction, ScenarioSettings{ }.throat_fraction );
    }
}

TEST_F( ScenarioTests, VenturiThroatIsAdjustable )
{
    settings.throat_fraction = 0.6F;

    for ( auto const scenario : { Scenario::Venturi, Scenario::LongVenturi } )
    {
        tunnel           = scenario_tunnel( scenario );
        auto const setup = make_scenario( scenario, tunnel, settings );

        EXPECT_NEAR( open_fraction( setup.solids, tunnel, tunnel.size.x * 0.5F ), 0.6F, 0.01F )
            << scenario_names[ static_cast< size_t >( scenario ) ];
    }
}

TEST_F( ScenarioTests, LongVenturiHasALongThroatAndGentleSlopes )
{
    auto const long_tunnel = scenario_tunnel( Scenario::LongVenturi );
    auto const long_setup  = make_scenario( Scenario::LongVenturi, long_tunnel, settings );
    auto const short_setup = make_scenario( Scenario::Venturi, tunnel, settings );

    // The tunnel itself is longer but no taller.
    EXPECT_GT( long_tunnel.size.x, tunnel.size.x );
    EXPECT_FLOAT_EQ( long_tunnel.size.y, tunnel.size.y );

    // The narrow section has a constant width for a quarter of the tunnel's length.
    EXPECT_NEAR(
        throat_length( long_setup.solids.polygons.front( ) ),
        long_tunnel.size.x * 0.25F,
        1.0e-4F
    );
    EXPECT_NEAR( open_fraction( long_setup.solids, long_tunnel, long_tunnel.size.x * 0.4F ), 1.0F / 3.0F, 0.01F );
    EXPECT_NEAR( open_fraction( long_setup.solids, long_tunnel, long_tunnel.size.x * 0.6F ), 1.0F / 3.0F, 0.01F );

    // The short venturi's throat is a single point.
    EXPECT_NEAR( throat_length( short_setup.solids.polygons.front( ) ), 0.0F, 1.0e-4F );

    // The walls are less than half as steep.
    auto const long_slope  = steepest_slope( long_setup.solids.polygons.front( ) );
    auto const short_slope = steepest_slope( short_setup.solids.polygons.front( ) );

    EXPECT_GT( long_slope, 0.0F );
    EXPECT_LT( long_slope, 0.5F * short_slope );
}

TEST_F( ScenarioTests, VenturiWallsAreMirrorImages )
{
    auto const walls = venturi_walls( tunnel, 1.0F, 0.5F, 0.8F, 0.4F );

    ASSERT_EQ( walls.size( ), 2UZ );
    ASSERT_EQ( walls[ 0 ].size( ), walls[ 1 ].size( ) );

    for ( auto i = 0UZ; i < walls[ 0 ].size( ); ++i )
    {
        EXPECT_FLOAT_EQ( walls[ 0 ][ i ].x, walls[ 1 ][ i ].x );
        EXPECT_NEAR( walls[ 0 ][ i ].y, tunnel.size.y - walls[ 1 ][ i ].y, 1.0e-5F );
    }

    auto const scene = SolidScene{ .polygons = walls };

    // Open before the walls start (x = 1) and after they end (x = 1 + 0.5 + 0.8 + 0.5).
    EXPECT_FLOAT_EQ( open_fraction( scene, tunnel, 0.95F ), 1.0F );
    EXPECT_FLOAT_EQ( open_fraction( scene, tunnel, 2.85F ), 1.0F );
    // A 0.4 high throat in a 2.0 high tunnel.
    EXPECT_NEAR( open_fraction( scene, tunnel, 1.6F ), 0.2F, 0.01F );
    EXPECT_NEAR( open_fraction( scene, tunnel, 2.2F ), 0.2F, 0.01F );
}

} // namespace
} // namespace ltb::flew
