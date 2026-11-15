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
};

constexpr auto scenario_names = std::array< char const*, 5 >{
    "Box",
    "Cylinder",
    "Airfoil",
    "Plate",
    "Venturi",
};

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
};

[[nodiscard( "Pure function" )]]
auto make_scenario( Scenario scenario, Tunnel const& tunnel, ScenarioSettings const& settings )
    -> ScenarioSetup;

} // namespace ltb::flew
