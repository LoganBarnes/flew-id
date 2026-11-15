// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "ltb/flew/scenarios.hpp"

// external
#include <gtest/gtest.h>

// standard
#include <algorithm>

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
};

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
        auto const* const name  = scenario_names[ static_cast< size_t >( scenario ) ];
        auto const        setup = make_scenario( scenario, tunnel, settings );

        EXPECT_FLOAT_EQ( setup.flow_speed, 0.7F ) << name;

        auto const fraction = solid_fraction( setup.solids, tunnel );
        EXPECT_GT( fraction, 0.001F ) << name;
        EXPECT_LT( fraction, 0.4F ) << name;

        // The inflow region on the left side of the tunnel must be clear of solids.
        for ( auto y = 0.05F; y < tunnel.size.y; y += 0.1F )
        {
            EXPECT_GT( signed_distance( setup.solids, { 0.1F, y } ), 0.1F ) << name;
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

    // Measure the open height at the narrowest point.
    auto open = 0;
    for ( auto i = 0; i < 1'000; ++i )
    {
        auto const y = ( static_cast< float32 >( i ) + 0.5F ) * tunnel.size.y / 1'000.0F;
        if ( signed_distance( setup.solids, { center_x, y } ) > 0.0F )
        {
            ++open;
        }
    }
    EXPECT_NEAR( static_cast< float32 >( open ) / 1'000.0F, 1.0F / 3.0F, 0.01F );

    // The middle is open and the constriction is symmetric about it.
    EXPECT_GT( signed_distance( setup.solids, { center_x, tunnel.size.y * 0.5F } ), 0.0F );
    EXPECT_LT( signed_distance( setup.solids, { center_x, tunnel.size.y * 0.1F } ), 0.0F );
    EXPECT_LT( signed_distance( setup.solids, { center_x, tunnel.size.y * 0.9F } ), 0.0F );
}

} // namespace
} // namespace ltb::flew
