// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "ltb/flew/solid_scene.hpp"

// external
#include <glm/gtc/constants.hpp>
#include <gtest/gtest.h>

// standard
#include <algorithm>
#include <cmath>

namespace ltb::flew
{
namespace
{

constexpr auto tolerance = 1.0e-5F;

/// \brief Twice the signed area of a polygon. Positive if the points are counter-clockwise.
auto doubled_signed_area( Polygon const& polygon ) -> float32
{
    auto area = 0.0F;
    for ( auto i = 0UZ, j = polygon.size( ) - 1UZ; i < polygon.size( ); j = i++ )
    {
        area += ( polygon[ j ].x * polygon[ i ].y ) - ( polygon[ i ].x * polygon[ j ].y );
    }
    return area;
}

TEST( SignedDistanceTests, EmptySceneIsFarFromEverything )
{
    EXPECT_GT( signed_distance( SolidScene{ }, { 1.0F, 1.0F } ), 1.0e5F );
}

TEST( SignedDistanceTests, Circle )
{
    auto const scene = SolidScene{ .circles = { { .center = { 1.0F, 1.0F }, .radius = 0.5F } } };

    EXPECT_NEAR( signed_distance( scene, { 2.0F, 1.0F } ), 0.5F, tolerance );
    EXPECT_NEAR( signed_distance( scene, { 1.0F, 1.5F } ), 0.0F, tolerance );
    EXPECT_NEAR( signed_distance( scene, { 1.0F, 1.0F } ), -0.5F, tolerance );
    EXPECT_NEAR( signed_distance( scene, { 1.25F, 1.0F } ), -0.25F, tolerance );
}

TEST( SignedDistanceTests, Polygon )
{
    auto const scene = SolidScene{ .polygons = { rectangle( { 0.0F, 0.0F }, { 2.0F, 1.0F } ) } };

    // Inside: the distance to the nearest edge (negative).
    EXPECT_NEAR( signed_distance( scene, { 1.0F, 0.5F } ), -0.5F, tolerance );
    EXPECT_NEAR( signed_distance( scene, { 0.1F, 0.5F } ), -0.1F, tolerance );
    // Outside: beside an edge and diagonally off a corner.
    EXPECT_NEAR( signed_distance( scene, { 3.0F, 0.5F } ), 1.0F, tolerance );
    EXPECT_NEAR( signed_distance( scene, { 1.0F, -2.0F } ), 2.0F, tolerance );
    EXPECT_NEAR( signed_distance( scene, { 3.0F, 2.0F } ), glm::root_two< float32 >( ), tolerance );
}

TEST( SignedDistanceTests, PolygonWindingDoesNotMatter )
{
    auto clockwise = rectangle( { 0.0F, 0.0F }, { 2.0F, 1.0F } );
    std::ranges::reverse( clockwise );
    auto const scene = SolidScene{ .polygons = { clockwise } };

    EXPECT_NEAR( signed_distance( scene, { 1.0F, 0.5F } ), -0.5F, tolerance );
    EXPECT_NEAR( signed_distance( scene, { 3.0F, 0.5F } ), 1.0F, tolerance );
}

TEST( SignedDistanceTests, NearestOfSeveralSolids )
{
    auto const scene = SolidScene{
        .polygons = { rectangle( { 0.0F, 0.0F }, { 1.0F, 1.0F } ) },
        .circles  = { { .center = { 4.0F, 0.5F }, .radius = 1.0F } },
    };

    // Closer to the rectangle, closer to the circle, and inside of each.
    EXPECT_NEAR( signed_distance( scene, { 1.5F, 0.5F } ), 0.5F, tolerance );
    EXPECT_NEAR( signed_distance( scene, { 2.5F, 0.5F } ), 0.5F, tolerance );
    EXPECT_LT( signed_distance( scene, { 0.5F, 0.5F } ), 0.0F );
    EXPECT_LT( signed_distance( scene, { 4.0F, 0.5F } ), 0.0F );
}

TEST( ShapeTests, Rectangle )
{
    auto const polygon = rectangle( { -1.0F, 2.0F }, { 3.0F, 5.0F } );

    ASSERT_EQ( polygon.size( ), 4UZ );
    // Counter-clockwise with the expected area (4 x 3).
    EXPECT_NEAR( doubled_signed_area( polygon ), 24.0F, tolerance );
}

TEST( ShapeTests, TransformedRotatesCounterClockwiseAboutThePivotThenMoves )
{
    auto const quarter_turn = glm::half_pi< float32 >( );

    auto const rotated = transformed( { { 1.0F, 0.0F } }, quarter_turn, { 0.0F, 0.0F }, { 0.0F, 0.0F } );
    EXPECT_NEAR( rotated[ 0 ].x, 0.0F, tolerance );
    EXPECT_NEAR( rotated[ 0 ].y, 1.0F, tolerance );

    // (3, 1) is (1, 0) from the pivot, which rotates to (0, 1) from the pivot.
    auto const moved = transformed( { { 3.0F, 1.0F } }, quarter_turn, { 2.0F, 1.0F }, { 10.0F, 20.0F } );
    EXPECT_NEAR( moved[ 0 ].x, 12.0F, tolerance );
    EXPECT_NEAR( moved[ 0 ].y, 22.0F, tolerance );
}

TEST( NacaAirfoilTests, SymmetricAirfoilHasTheRequestedSizeAndThickness )
{
    constexpr auto chord           = 2.0F;
    constexpr auto points_per_side = 100U;

    auto const polygon = naca_airfoil( 12U, chord, points_per_side );

    ASSERT_EQ( polygon.size( ), 2UZ * points_per_side );
    EXPECT_GT( doubled_signed_area( polygon ), 0.0F ) << "Should be counter-clockwise";

    auto min_corner = polygon.front( );
    auto max_corner = polygon.front( );
    for ( auto const& point : polygon )
    {
        min_corner = glm::min( min_corner, point );
        max_corner = glm::max( max_corner, point );
    }

    // Leading edge at the origin, trailing edge at the chord.
    EXPECT_NEAR( min_corner.x, 0.0F, 1.0e-4F );
    EXPECT_NEAR( max_corner.x, chord, 1.0e-4F );
    // A "12" airfoil is 12% as thick as it is long, and symmetric about the chord line.
    EXPECT_NEAR( max_corner.y - min_corner.y, 0.12F * chord, 0.002F * chord );
    EXPECT_NEAR( max_corner.y, -min_corner.y, 1.0e-4F );
}

TEST( NacaAirfoilTests, CamberedAirfoilCurvesUpward )
{
    constexpr auto chord = 1.0F;

    auto const scene = SolidScene{ .polygons = { naca_airfoil( 4412U, chord ) } };

    // The camber line of a 4412 peaks at 4% of the chord, 40% of the way along it.
    EXPECT_LT( signed_distance( scene, { 0.4F, 0.04F } ), 0.0F );
    // The upper surface is further from the chord line than the lower surface.
    EXPECT_LT( signed_distance( scene, { 0.4F, 0.08F } ), 0.0F );
    EXPECT_GT( signed_distance( scene, { 0.4F, -0.08F } ), 0.0F );
    // Outside in front of and behind the airfoil.
    EXPECT_GT( signed_distance( scene, { -0.01F, 0.0F } ), 0.0F );
    EXPECT_GT( signed_distance( scene, { 1.01F, 0.0F } ), 0.0F );
}

class DistanceFieldTests : public ::testing::Test
{
protected:
    Tunnel        tunnel     = { .size = { 4.0F, 2.0F } };
    glm::ivec2    size       = { 256, 128 };
    glm::vec2     texel_size = tunnel.size / glm::vec2( size );
    SolidScene    scene      = { .circles = { { .center = { 1.5F, 1.0F }, .radius = 0.4F } } };
    DistanceField field      = build_distance_field( scene, tunnel, size );

    [[nodiscard( "Const getter" )]]
    auto texel_center( int32 const x, int32 const y ) const -> glm::vec2
    {
        return ( glm::vec2( glm::ivec2{ x, y } ) + 0.5F ) * texel_size;
    }

    [[nodiscard( "Const getter" )]]
    auto texel( int32 const x, int32 const y ) const -> glm::vec4 const&
    {
        return field.texels[ ( static_cast< size_t >( y ) * static_cast< size_t >( size.x ) ) + static_cast< size_t >( x ) ];
    }
};

TEST_F( DistanceFieldTests, HasOneTexelPerCell )
{
    EXPECT_EQ( field.size, size );
    EXPECT_EQ( field.texels.size( ), static_cast< size_t >( size.x ) * static_cast< size_t >( size.y ) );
}

TEST_F( DistanceFieldTests, IsExactNearSurfacesAndNeverOverestimatesElsewhere )
{
    // Distances are only required to be exact within a few texels of a surface.
    auto const near = 4.0F * texel_size.x;

    auto near_count = 0U;
    for ( auto y = 0; y < size.y; ++y )
    {
        for ( auto x = 0; x < size.x; ++x )
        {
            auto const exact  = signed_distance( scene, texel_center( x, y ) );
            auto const stored = texel( x, y ).x;

            if ( std::abs( exact ) < near )
            {
                ASSERT_NEAR( stored, exact, tolerance ) << "Texel " << x << ", " << y;
                ++near_count;
            }
            else if ( exact > 0.0F )
            {
                // Far outside: positive, but no further than the surface really is.
                ASSERT_GT( stored, 0.0F ) << "Texel " << x << ", " << y;
                ASSERT_LE( stored, exact + tolerance ) << "Texel " << x << ", " << y;
            }
            else
            {
                ASSERT_LT( stored, 0.0F ) << "Texel " << x << ", " << y;
            }
        }
    }
    EXPECT_GT( near_count, 100U );
}

TEST_F( DistanceFieldTests, GradientPointsAwayFromTheSurface )
{
    auto const& circle = scene.circles.front( );

    auto checked = 0U;
    for ( auto y = 0; y < size.y; ++y )
    {
        for ( auto x = 0; x < size.x; ++x )
        {
            auto const center = texel_center( x, y );
            auto const exact  = signed_distance( scene, center );

            if ( std::abs( exact ) < ( 2.0F * texel_size.x ) )
            {
                auto const expected = glm::normalize( center - circle.center );
                auto const gradient = glm::vec2{ texel( x, y ).y, texel( x, y ).z };

                ASSERT_NEAR( glm::length( gradient ), 1.0F, 1.0e-4F );
                ASSERT_GT( glm::dot( gradient, expected ), 0.99F ) << "Texel " << x << ", " << y;
                ++checked;
            }
        }
    }
    EXPECT_GT( checked, 50U );
}

TEST_F( DistanceFieldTests, EmptySceneHasNoSolidTexels )
{
    auto const empty = build_distance_field( SolidScene{ }, tunnel, size );

    EXPECT_TRUE( std::ranges::all_of( empty.texels, []( glm::vec4 const& t ) { return t.x > 0.0F; } ) );
}

TEST_F( DistanceFieldTests, SampleDistanceMatchesTexelCentersAndInterpolatesBetweenThem )
{
    // Exactly the stored value at the center of a texel.
    EXPECT_NEAR( sample_distance( field, tunnel, texel_center( 100, 60 ) ), texel( 100, 60 ).x, tolerance );

    // Close to the exact distance at arbitrary points near the surface.
    auto const& circle = scene.circles.front( );
    for ( auto const angle : { 0.0F, 0.7F, 2.0F, 3.5F, 5.1F } )
    {
        auto const direction = glm::vec2{ std::cos( angle ), std::sin( angle ) };

        auto const on_surface = circle.center + ( direction * circle.radius );
        EXPECT_NEAR( sample_distance( field, tunnel, on_surface ), 0.0F, 0.1F * texel_size.x );

        auto const outside = circle.center + ( direction * ( circle.radius + 0.03F ) );
        EXPECT_NEAR( sample_distance( field, tunnel, outside ), 0.03F, 0.1F * texel_size.x );

        auto const inside = circle.center + ( direction * ( circle.radius - 0.03F ) );
        EXPECT_NEAR( sample_distance( field, tunnel, inside ), -0.03F, 0.1F * texel_size.x );
    }
}

TEST_F( DistanceFieldTests, SamplingOutsideTheTunnelClampsToTheEdge )
{
    EXPECT_NEAR( sample_distance( field, tunnel, { -5.0F, -5.0F } ), texel( 0, 0 ).x, tolerance );
    EXPECT_NEAR(
        sample_distance( field, tunnel, tunnel.size + 5.0F ),
        texel( size.x - 1, size.y - 1 ).x,
        tolerance
    );
}

} // namespace
} // namespace ltb::flew
