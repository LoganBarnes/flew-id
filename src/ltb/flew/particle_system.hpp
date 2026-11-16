// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#pragma once

// project
#include "ltb/flew/solid_scene.hpp"
#include "ltb/vlk/command_pool.hpp"
#include "ltb/vlk/objs/vulkan_buffer.hpp"
#include "ltb/vlk/objs/vulkan_compute_pipeline.hpp"
#include "ltb/vlk/objs/vulkan_gpu.hpp"

namespace ltb::flew
{

struct ParticleSystemSettings
{
    Tunnel               tunnel = { };
    ParticleFillSettings fill   = { };
    SolidScene           solids = { };
    /// \brief The resolution of the distance field used to collide with the solids.
    glm::ivec2 solid_field_size = { 1024, 512 };
    /// \brief The flow fields are measured on a coarser grid than the one used for collisions.
    ///        Each field cell covers this many collision cells in each direction.
    uint32 field_cell_scale = 6U;
};

/// \brief Values that can change between simulation steps.
struct StepParams
{
    float32 delta_time = 1.0e-3F;
    /// \brief Spring constant of the repulsion between overlapping particles (unit mass).
    float32 stiffness = 1.0e4F;
    /// \brief Rate (1/s) at which touching particles share tangential velocity.
    float32 viscosity = 0.0F;
    /// \brief Rate (1/s) at which touching particles lose relative normal
    ///        velocity. Zero is perfectly elastic.
    float32 damping = 0.0F;

    /// \brief The speed of the flow entering the left side of the tunnel. When zero the
    ///        tunnel is a closed box, otherwise particles leaving the right side re-enter
    ///        on the left.
    float32 flow_speed = 0.0F;
    /// \brief The root mean square speed of the random motion given to entering particles.
    float32 thermal_speed = 0.5F;
    /// \brief The width of the region on the left side of the tunnel where
    ///        particles are reset to the incoming flow conditions.
    float32 inflow_width = 0.1F;
    /// \brief How often (1/s) each particle in the inflow region is reset.
    float32 inflow_rate = 50.0F;

    /// \brief Fraction of tangential velocity lost when touching a
    ///        solid. 0 is a frictionless (slip) surface, 1 is no-slip.
    float32 wall_friction = 0.5F;
    /// \brief Fraction of normal speed kept when bouncing off a solid.
    float32 wall_restitution = 1.0F;

    /// \brief How much of each step's measurement of the flow fields is blended into their
    ///        running average [0, 1]. Smaller values are smoother but respond more slowly.
    float32 field_blend = 0.002F;

    /// \brief How particles leaving either end of a flowing tunnel are treated. When false a
    ///        particle keeps its velocity as it re-enters at the other end (it is only
    ///        gradually reset in the inflow region). When true it is replaced by a new
    ///        particle of the undisturbed incoming flow, as if the tunnel continued far
    ///        beyond both ends.
    bool respawn = false;

    /// \brief Surround the ends of a flowing tunnel with buffer zones, like the settling
    ///        chamber and diffuser of a real wind tunnel. Particles inside a buffer are
    ///        randomly reset to the undisturbed flow, more often the closer they are to the
    ///        end of the tunnel. The outlet buffer absorbs wakes and pressure waves before
    ///        they can leave, and the inlet buffer delivers a uniform flow. Buffers replace
    ///        the inflow region (`inflow_width`) and always respawn particles at the ends.
    bool buffers = true;
    /// \brief The length of the buffer zone at each end of the tunnel.
    float32 buffer_width = 0.3F;
};

/// \brief Step parameters that keep the simulation stable for particles of the given size.
///
/// The time step limits a particle moving at `typical_speed` to a fraction of its radius
/// per step, and the stiffness is the largest the time step can integrate accurately.
[[nodiscard( "Pure function" )]]
auto default_step_params( float32 radius, float32 typical_speed ) -> StepParams;

/// \brief Owns the particle data on the GPU and the compute passes that update it.
///
/// Each step sorts the particles into a uniform grid (counting sort) and then
/// resolves collisions between particles in neighboring cells:
///
///   1. clear the per-cell counters
///   2. count the particles in each cell
///   3. prefix sum the counters to get the first particle index of each cell
///   4. scatter the particles into a second buffer ordered by cell
///   5. collide and integrate, writing the result back to the first buffer
class ParticleSystem
{
public:
    explicit( false ) ParticleSystem( vlk::objs::VulkanGpu& gpu );

    /// \brief Creates the GPU resources and uploads the initial particles.
    auto initialize(
        ParticleSystemSettings settings,
        vlk::CommandPool&      command_pool,
        vk::Queue const&       queue
    ) -> utils::Result< void >;

    [[nodiscard( "Const getter" )]]
    auto is_initialized( ) const -> bool;

    /// \brief Records the commands required to advance the simulation by one step.
    auto record_step( vk::CommandBuffer const& command_buffer, StepParams const& params ) -> void;

    /// \brief Waits for the GPU to finish and copies the current particles back to the CPU.
    auto read_particles( vlk::CommandPool& command_pool, vk::Queue const& queue )
        -> utils::Result< std::vector< Particle > >;

    /// \brief The buffer containing the current particles (usable as a vertex buffer).
    [[nodiscard( "Const getter" )]]
    auto particle_buffer( ) const -> vk::Buffer const&;

    /// \brief The buffer containing the distance field of the solids (see `solid.glsl`).
    [[nodiscard( "Const getter" )]]
    auto solid_buffer( ) const -> vlk::objs::VulkanBuffer const&;

    /// \brief The buffer containing the time averaged flow fields. One `vec4` per field
    ///        cell: packing fraction, mean velocity (x, y), and temperature.
    [[nodiscard( "Const getter" )]]
    auto field_buffer( ) const -> vlk::objs::VulkanBuffer const&;

    [[nodiscard( "Const getter" )]] auto field_size( ) const -> glm::ivec2 const&;
    [[nodiscard( "Const getter" )]] auto particle_count( ) const -> uint32;
    [[nodiscard( "Const getter" )]] auto radius( ) const -> float32;
    [[nodiscard( "Const getter" )]] auto tunnel( ) const -> Tunnel const&;
    [[nodiscard( "Const getter" )]] auto grid_size( ) const -> glm::ivec2 const&;
    [[nodiscard( "Const getter" )]] auto settings( ) const -> ParticleSystemSettings const&;

private:
    /// \brief One level of the prefix sum hierarchy stored in the grid buffer.
    struct ScanLevel
    {
        uint32 offset = 0U;
        uint32 count  = 0U;
    };

    vlk::objs::VulkanGpu& gpu_;

    ParticleSystemSettings settings_  = { };
    float32                radius_    = 0.0F;
    glm::ivec2             grid_size_ = { };

    /// \brief The cells, the block sums of the cells, the block
    ///        sums of those sums, and a single unused total.
    std::array< ScanLevel, 4 > scan_levels_ = { };

    /// \brief Two ranges: the current particles (A) and the cell sorted particles (B).
    vlk::objs::VulkanBuffer particles_ = { gpu_ };
    vlk::objs::VulkanBuffer grid_      = { gpu_ };
    /// \brief The distance field of the solids.
    vlk::objs::VulkanBuffer solids_ = { gpu_ };

    /// \brief The time averaged flow fields.
    vlk::objs::VulkanBuffer fields_     = { gpu_ };
    glm::ivec2              field_size_ = { };

    uint32 step_index_ = 0U;

    vlk::objs::VulkanComputePipeline count_pipeline_     = { gpu_ };
    vlk::objs::VulkanComputePipeline scan_up_pipeline_   = { gpu_ };
    vlk::objs::VulkanComputePipeline scan_down_pipeline_ = { gpu_ };
    vlk::objs::VulkanComputePipeline scatter_pipeline_   = { gpu_ };
    vlk::objs::VulkanComputePipeline collide_pipeline_   = { gpu_ };
    vlk::objs::VulkanComputePipeline fields_pipeline_    = { gpu_ };

    bool initialized_ = false;

    auto initialize_buffers( std::vector< Particle > const& particles, DistanceField const& field, vlk::CommandPool& command_pool, vk::Queue const& queue )
        -> utils::Result< void >;
    auto initialize_pipeline( vlk::objs::VulkanComputePipeline& pipeline, std::string_view shader_name )
        -> utils::Result< void >;
};

} // namespace ltb::flew
