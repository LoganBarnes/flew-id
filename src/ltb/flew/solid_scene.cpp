// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "ltb/flew/solid_scene.hpp"

// external
#include <glm/gtc/constants.hpp>

// standard
#include <algorithm>
#include <execution>
#include <ranges>
#include <cmath>
#include <limits>

namespace ltb::flew
{
namespace
{

/// \brief The distance reported when there is nothing nearby.
constexpr auto far_distance = 1.0e6F;

auto polygon_signed_distance( Polygon const& polygon, glm::vec2 const& point ) -> float32
{
    if ( polygon.size( ) < 3UZ )
    {
        return far_distance;
    }

    auto min_distance_sq = std::numeric_limits< float32 >::max( );
    auto inside          = false;

    for ( auto i = 0UZ, j = polygon.size( ) - 1UZ; i < polygon.size( ); j = i++ )
    {
        auto const& a = polygon[ j ];
        auto const& b = polygon[ i ];

        // Distance to the edge.
        auto const edge     = b - a;
        auto const to_point = point - a;
        auto const t = std::clamp( glm::dot( to_point, edge ) / glm::dot( edge, edge ), 0.0F, 1.0F );
        auto const nearest = to_point - ( edge * t );
        min_distance_sq    = std::min( min_distance_sq, glm::dot( nearest, nearest ) );

        // Even-odd rule: count the edges crossed by a ray in the +x direction.
        if ( ( a.y > point.y ) != ( b.y > point.y ) )
        {
            auto const x_at_y = a.x + ( ( point.y - a.y ) * ( b.x - a.x ) / ( b.y - a.y ) );
            if ( point.x < x_at_y )
            {
                inside = !inside;
            }
        }
    }

    auto const dist = std::sqrt( min_distance_sq );
    return inside ? -dist : dist;
}

} // namespace

auto signed_distance( SolidScene const& scene, glm::vec2 const& point ) -> float32
{
    auto result = far_distance;

    for ( auto const& polygon : scene.polygons )
    {
        result = std::min( result, polygon_signed_distance( polygon, point ) );
    }
    for ( auto const& circle : scene.circles )
    {
        result = std::min( result, glm::distance( point, circle.center ) - circle.radius );
    }
    return result;
}

auto build_distance_field( SolidScene const& scene, Tunnel const& tunnel, glm::ivec2 const size )
    -> DistanceField
{
    auto const width  = static_cast< size_t >( size.x );
    auto const height = static_cast< size_t >( size.y );

    auto const texel_size = tunnel.size / glm::vec2( size );

    // Exact distances are only needed close to the solids (particles only react when they
    // touch a surface). Everywhere else is given this distance, which is far cheaper than
    // measuring the distance to every edge of every solid from every texel.
    auto const band = 8.0F * glm::max( texel_size.x, texel_size.y );

    auto field = DistanceField{
        .size   = size,
        .texels = std::vector< glm::vec4 >( width * height, glm::vec4{ band, 0.0F, 1.0F, 0.0F } ),
    };

    auto const rows = std::views::iota( 0UZ, height );

    // Writes the distance to one shape into the texels within `band` of its bounding box.
    auto const add_shape = [ & ]( glm::vec2 const& min_corner, glm::vec2 const& max_corner, auto const& distance_to ) {
        auto const lower = glm::max( glm::ivec2( glm::floor( ( min_corner - band ) / texel_size ) ), glm::ivec2( 0 ) );
        auto const upper = glm::min( glm::ivec2( glm::ceil( ( max_corner + band ) / texel_size ) ), size );
        if ( lower.x >= upper.x || lower.y >= upper.y )
        {
            return;
        }

        std::for_each( std::execution::par, rows.begin( ), rows.end( ), [ & ]( size_t const y ) {
            if ( y < static_cast< size_t >( lower.y ) || y >= static_cast< size_t >( upper.y ) )
            {
                return;
            }
            for ( auto x = static_cast< size_t >( lower.x ); x < static_cast< size_t >( upper.x ); ++x )
            {
                auto const point = ( glm::vec2( glm::uvec2{ x, y } ) + 0.5F ) * texel_size;
                auto&      texel = field.texels[ ( y * width ) + x ];
                texel.x          = std::min( texel.x, std::min( distance_to( point ), band ) );
            }
        } );
    };

    for ( auto const& polygon : scene.polygons )
    {
        if ( polygon.size( ) < 3UZ )
        {
            continue;
        }
        auto min_corner = polygon.front( );
        auto max_corner = polygon.front( );
        for ( auto const& point : polygon )
        {
            min_corner = glm::min( min_corner, point );
            max_corner = glm::max( max_corner, point );
        }
        add_shape( min_corner, max_corner, [ & ]( glm::vec2 const& point ) {
            return polygon_signed_distance( polygon, point );
        } );
    }
    for ( auto const& circle : scene.circles )
    {
        add_shape( circle.center - circle.radius, circle.center + circle.radius, [ & ]( glm::vec2 const& point ) {
            return glm::distance( point, circle.center ) - circle.radius;
        } );
    }

    // The direction away from the surface is the gradient of the distance.
    auto const distance_at = [ & ]( size_t const x, size_t const y ) {
        return field.texels[ ( y * width ) + x ].x;
    };
    for ( auto y = 0UZ; y < height; ++y )
    {
        for ( auto x = 0UZ; x < width; ++x )
        {
            auto const x0 = ( x > 0UZ ) ? x - 1UZ : x;
            auto const x1 = std::min( x + 1UZ, width - 1UZ );
            auto const y0 = ( y > 0UZ ) ? y - 1UZ : y;
            auto const y1 = std::min( y + 1UZ, height - 1UZ );

            auto gradient = glm::vec2{
                ( distance_at( x1, y ) - distance_at( x0, y ) ) / ( static_cast< float32 >( x1 - x0 ) * texel_size.x ),
                ( distance_at( x, y1 ) - distance_at( x, y0 ) ) / ( static_cast< float32 >( y1 - y0 ) * texel_size.y ),
            };
            auto const length = glm::length( gradient );
            gradient          = ( length > 1.0e-6F ) ? gradient / length : glm::vec2{ 0.0F, 1.0F };

            auto& texel = field.texels[ ( y * width ) + x ];
            texel.y     = gradient.x;
            texel.z     = gradient.y;
        }
    }

    return field;
}

auto sample_distance( DistanceField const& field, Tunnel const& tunnel, glm::vec2 const& point )
    -> float32
{
    auto const coords = ( point / tunnel.size * glm::vec2( field.size ) ) - 0.5F;
    auto const lower  = glm::ivec2( glm::floor( coords ) );
    auto const t      = coords - glm::vec2( lower );

    auto const max_coords = field.size - 1;
    auto const c00        = glm::clamp( lower, glm::ivec2( 0 ), max_coords );
    auto const c11        = glm::clamp( lower + 1, glm::ivec2( 0 ), max_coords );

    auto const at = [ & ]( int32 const x, int32 const y ) {
        return field.texels[ ( static_cast< size_t >( y ) * static_cast< size_t >( field.size.x ) ) + static_cast< size_t >( x ) ].x;
    };

    return glm::mix(
        glm::mix( at( c00.x, c00.y ), at( c11.x, c00.y ), t.x ),
        glm::mix( at( c00.x, c11.y ), at( c11.x, c11.y ), t.x ),
        t.y
    );
}

auto naca_airfoil( uint32 const digits, float32 const chord, uint32 const points_per_side )
    -> Polygon
{
    // https://en.wikipedia.org/wiki/NACA_airfoil#Four-digit_series
    auto const max_camber      = static_cast< float32 >( ( digits / 1000U ) % 10U ) / 100.0F;
    auto const camber_position = static_cast< float32 >( ( digits / 100U ) % 10U ) / 10.0F;
    auto const thickness       = static_cast< float32 >( digits % 100U ) / 100.0F;

    auto upper = Polygon{ };
    auto lower = Polygon{ };

    for ( auto i = 0U; i <= points_per_side; ++i )
    {
        // Cosine spacing places more points near the leading and trailing edges.
        auto const beta = glm::pi< float32 >( ) * static_cast< float32 >( i )
                        / static_cast< float32 >( points_per_side );
        auto const x = 0.5F * ( 1.0F - std::cos( beta ) );

        // Half thickness (closed trailing edge).
        auto const yt = 5.0F * thickness
                      * ( ( 0.2969F * std::sqrt( x ) ) - ( 0.1260F * x ) - ( 0.3516F * x * x )
                          + ( 0.2843F * x * x * x ) - ( 0.1036F * x * x * x * x ) );

        auto yc    = 0.0F;
        auto slope = 0.0F;
        if ( max_camber > 0.0F && camber_position > 0.0F )
        {
            auto const p = camber_position;
            if ( x < p )
            {
                yc    = max_camber / ( p * p ) * ( ( 2.0F * p * x ) - ( x * x ) );
                slope = 2.0F * max_camber / ( p * p ) * ( p - x );
            }
            else
            {
                auto const q = 1.0F - p;
                yc    = max_camber / ( q * q ) * ( ( 1.0F - ( 2.0F * p ) ) + ( 2.0F * p * x ) - ( x * x ) );
                slope = 2.0F * max_camber / ( q * q ) * ( p - x );
            }
        }
        auto const theta = std::atan( slope );

        upper.push_back( glm::vec2{ x - ( yt * std::sin( theta ) ), yc + ( yt * std::cos( theta ) ) } * chord );
        lower.push_back( glm::vec2{ x + ( yt * std::sin( theta ) ), yc - ( yt * std::cos( theta ) ) } * chord );
    }

    // Counter-clockwise: trailing edge to leading edge along the top, then back along the bottom.
    auto polygon = Polygon{ upper.rbegin( ), upper.rend( ) };
    polygon.insert( polygon.end( ), lower.begin( ) + 1, lower.end( ) - 1 );
    return polygon;
}

auto rectangle( glm::vec2 const& min_corner, glm::vec2 const& max_corner ) -> Polygon
{
    return {
        { min_corner.x, min_corner.y },
        { max_corner.x, min_corner.y },
        { max_corner.x, max_corner.y },
        { min_corner.x, max_corner.y },
    };
}

auto transformed(
    Polygon          polygon,
    float32 const    angle_radians,
    glm::vec2 const& pivot,
    glm::vec2 const& offset
) -> Polygon
{
    auto const c = std::cos( angle_radians );
    auto const s = std::sin( angle_radians );

    for ( auto& point : polygon )
    {
        auto const local = point - pivot;
        point = pivot + offset + glm::vec2{ ( c * local.x ) - ( s * local.y ), ( s * local.x ) + ( c * local.y ) };
    }
    return polygon;
}

} // namespace ltb::flew
