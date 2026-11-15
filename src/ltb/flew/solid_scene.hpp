// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#pragma once

// project
#include "ltb/flew/particle.hpp"

namespace ltb::flew
{

/// \brief A closed loop of points. The last point connects back to the first.
using Polygon = std::vector< glm::vec2 >;

struct Circle
{
    glm::vec2 center = { };
    float32   radius = 0.0F;
};

/// \brief The solid objects particles collide with.
struct SolidScene
{
    std::vector< Polygon > polygons = { };
    std::vector< Circle >  circles  = { };
};

/// \brief The distance from a point to the surface of the nearest solid.
///        Negative inside a solid and a large positive value if the scene is empty.
[[nodiscard( "Pure function" )]]
auto signed_distance( SolidScene const& scene, glm::vec2 const& point ) -> float32;

/// \brief A grid of signed distances covering the tunnel. Matches `solid.glsl`.
struct DistanceField
{
    glm::ivec2 size = { };
    /// \brief Row major. x is the signed distance at the center of the
    ///        texel and yz is the direction away from the nearest surface.
    ///        Distances are only exact near the solids. Far from them the
    ///        distance is a positive value no larger than the true distance.
    std::vector< glm::vec4 > texels = { };
};

[[nodiscard( "Pure function" )]]
auto build_distance_field( SolidScene const& scene, Tunnel const& tunnel, glm::ivec2 size )
    -> DistanceField;

/// \brief The (bilinearly interpolated) signed distance stored in a distance field.
[[nodiscard( "Pure function" )]]
auto sample_distance( DistanceField const& field, Tunnel const& tunnel, glm::vec2 const& point )
    -> float32;

/// \brief The outline of a NACA 4-digit airfoil (e.g. 2412) with its leading edge at the
///        origin and its trailing edge at `(chord, 0)`.
[[nodiscard( "Pure function" )]]
auto naca_airfoil( uint32 digits, float32 chord, uint32 points_per_side = 60U ) -> Polygon;

[[nodiscard( "Pure function" )]]
auto rectangle( glm::vec2 const& min_corner, glm::vec2 const& max_corner ) -> Polygon;

/// \brief Rotates a polygon counter-clockwise about `pivot` then moves it by `offset`.
[[nodiscard( "Pure function" )]]
auto transformed(
    Polygon          polygon,
    float32          angle_radians,
    glm::vec2 const& pivot,
    glm::vec2 const& offset
) -> Polygon;

} // namespace ltb::flew
