// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#pragma once

// project
#include "ltb/flew/solid_scene.hpp"

// standard
#include <array>
#include <string_view>

namespace ltb::flew
{

enum class Scenario : uint32
{
    /// \brief A closed box of gas with no flow.
    Box,
    /// \brief Flow around a circle (vortex shedding).
    Cylinder,
    /// \brief Flow around a cambered wing at an angle of attack (lift).
    Airfoil,
    /// \brief Flow around a steeply inclined flat plate (stall).
    Plate,
    /// \brief Flow through a constriction (Bernoulli).
    Venturi,
    /// \brief A longer tunnel with gently sloped walls leading to a long narrow section.
    LongVenturi,
};

constexpr auto scenario_names = std::array< char const*, 6 >{
    "Box",
    "Cylinder",
    "Airfoil",
    "Plate",
    "Venturi",
    "LongVenturi",
};

/// \brief The size of the tunnel a scenario is designed for.
[[nodiscard( "Pure function" )]]
auto scenario_tunnel( Scenario scenario ) -> Tunnel;

/// \brief Finds a scenario by (case sensitive) name.
[[nodiscard( "Pure function" )]]
auto scenario_from_name( std::string_view name ) -> std::optional< Scenario >;

struct ScenarioSetup
{
    SolidScene solids = { };
    /// \brief The speed of the flow entering the left side of the tunnel. With no flow the
    ///        tunnel is a closed box. Otherwise particles leaving the right side re-enter
    ///        on the left.
    float32 flow_speed = 0.0F;
};

struct ScenarioSettings
{
    /// \brief Angle of attack of the airfoil and plate scenarios.
    float32 angle_degrees = 8.0F;
    float32 flow_speed    = 0.5F;
    /// \brief The fraction of the tunnel's height left open at the
    ///        narrowest part of the venturi scenarios.
    float32 throat_fraction = 1.0F / 3.0F;
};

/// \brief The walls of a venturi: the top and bottom of the tunnel smoothly narrow over
///        `ramp_length`, stay `throat_height` apart for `throat_length`, then widen again.
[[nodiscard( "Pure function" )]]
auto venturi_walls(
    Tunnel const& tunnel,
    float32       start_x,
    float32       ramp_length,
    float32       throat_length,
    float32       throat_height
) -> std::vector< Polygon >;

[[nodiscard( "Pure function" )]]
auto make_scenario( Scenario scenario, Tunnel const& tunnel, ScenarioSettings const& settings )
    -> ScenarioSetup;

} // namespace ltb::flew
