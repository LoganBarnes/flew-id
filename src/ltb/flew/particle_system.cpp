// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "ltb/flew/particle_system.hpp"

// project
#include "ltb/flew/ltb_flew_config.hpp"
#include "ltb/vlk/buffer_utils.hpp"
#include "ltb/vlk/check.hpp"
#include "ltb/vlk/command_buffer.hpp"
#include "ltb/vlk/device_memory_utils.hpp"

// external
#include <glm/gtc/constants.hpp>

// standard
#include <algorithm>
#include <cmath>
#include <cstring>

namespace ltb::flew
{
namespace
{

/// \brief Matches `SCAN_BLOCK_SIZE` in `particle.glsl`.
constexpr auto scan_block_size = 1024U;
/// \brief Matches `SCAN_THREADS` in `grid_scan_up.comp`.
constexpr auto scan_threads = 256U;

/// \brief SPH grid cells are this fraction of the kernel's support wide. Matches
///        `SPH_CELL_REACH` in `sph.glsl` (the number of cells searched in each direction).
constexpr auto sph_cell_subdivision = 2;

/// \brief More than the number of timestamps written in a single step.
constexpr auto max_timestamps = 32U;

/// \brief Matches `StepConstants` in `particle.glsl`.
struct StepConstants
{
    glm::vec2 tunnel_size = { };
    float32   delta_time  = 0.0F;
    float32   radius      = 0.0F;

    glm::ivec2 grid_size      = { };
    uint32     particle_count = 0U;
    float32    stiffness      = 0.0F;

    float32 viscosity         = 0.0F;
    float32 damping           = 0.0F;
    uint32  scan_child_offset = 0U;
    uint32  scan_child_count  = 0U;

    uint32     scan_parent_offset = 0U;
    uint32     step_index         = 0U;
    glm::ivec2 solid_size         = { };

    float32 flow_speed    = 0.0F;
    float32 thermal_speed = 0.0F;
    float32 inflow_width  = 0.0F;
    float32 inflow_rate   = 0.0F;

    float32 wall_friction    = 0.0F;
    float32 wall_restitution = 1.0F;

    glm::ivec2 field_size  = { };
    uint32     field_scale = 1U;
    float32    field_blend = 0.0F;

    uint32 boundary_flags = 0U;

    uint32  model            = 0U;
    float32 sound_speed      = 0.0F;
    float32 smoothing_length = 0.0F;
    float32 particle_mass    = 0.0F;
    float32 rest_density     = 1.0F;
};

// Vulkan only guarantees 128 bytes of push constants.
static_assert( sizeof( StepConstants ) <= 128UZ );

/// \brief Matches RESPAWN_FLAG and BUFFERS_FLAG in `particle.glsl`.
constexpr auto respawn_flag = 1U;
constexpr auto buffers_flag = 2U;

constexpr auto particles_a_range = 0UZ;
constexpr auto particles_b_range = 1UZ;

constexpr auto group_count( uint32 const item_count, uint32 const group_size ) -> uint32
{
    return ( item_count + group_size - 1U ) / group_size;
}

/// \brief Makes all previous writes visible to all following
///        reads and writes (compute, transfer, and vertex input).
auto full_barrier( vk::CommandBuffer const& command_buffer ) -> void
{
    constexpr auto stages = vk::PipelineStageFlagBits::eComputeShader
                          | vk::PipelineStageFlagBits::eTransfer
                          | vk::PipelineStageFlagBits::eVertexInput;

    constexpr auto access = vk::AccessFlagBits::eShaderRead
                          | vk::AccessFlagBits::eShaderWrite
                          | vk::AccessFlagBits::eTransferRead
                          | vk::AccessFlagBits::eTransferWrite
                          | vk::AccessFlagBits::eVertexAttributeRead;

    auto const barrier = vk::MemoryBarrier{ }.setSrcAccessMask( access ).setDstAccessMask( access );

    command_buffer.pipelineBarrier( stages, stages, { }, barrier, { }, { } );
}

} // namespace

auto default_step_params( float32 const radius, float32 const typical_speed ) -> StepParams
{
    // The fraction of its radius a typical particle moves in one step.
    constexpr auto step_fraction = 0.05F;
    // The phase (radians) of a pair's spring oscillation covered by one step.
    constexpr auto phase_per_step = 0.3F;

    auto const delta_time = step_fraction * radius / typical_speed;
    // A pair of unit masses joined by a spring oscillates at sqrt(2 * stiffness).
    auto const frequency = phase_per_step / delta_time;

    return StepParams{
        .delta_time = delta_time,
        .stiffness  = 0.5F * frequency * frequency,
    };
}

auto default_sph_step_params( float32 const smoothing_length, float32 const flow_speed )
    -> StepParams
{
    // The flow speeds up as it squeezes around solids.
    constexpr auto speed_up = 1.5F;
    // Used when there is no flow (a closed box).
    constexpr auto minimum_speed = 0.25F;
    // The ratio of the speed of sound to the fastest flow speed.
    constexpr auto sound_ratio = 10.0F;
    // The fraction of the smoothing length a sound wave may cross in one step.
    constexpr auto courant_number = 0.25F;
    // The fields are averaged over the time sound takes to travel this far (the height
    // of the tunnel), which smooths out sound waves bouncing between its walls.
    constexpr auto field_average_distance = 2.0F;

    auto const max_speed   = std::max( flow_speed * speed_up, minimum_speed );
    auto const sound_speed = sound_ratio * max_speed;

    auto params = StepParams{
        .delta_time = courant_number * smoothing_length / ( sound_speed + max_speed ),
        .stiffness  = 0.0F,
        // A kinematic viscosity giving thin boundary layers that are still a few
        // particles thick at a few hundred thousand particles.
        .viscosity  = 1.0e-3F,
        .flow_speed = flow_speed,
        // No random motion.
        .thermal_speed = 0.0F,
        // Fully no-slip solids, and no bouncing.
        .wall_friction    = 1.0F,
        .wall_restitution = 0.0F,
        .sound_speed      = sound_speed,
    };
    // The pressure of a stiff fluid is noisy: a tiny difference in density is a large
    // difference in pressure, so sound waves easily hide the pressure of the flow.
    auto const field_average_time = field_average_distance / sound_speed;
    params.field_blend            = std::min( params.delta_time / field_average_time, 1.0F );

    // The background pressure is a few times the dynamic pressure of the flow: enough
    // to keep the pressure positive in the low pressure regions around solids.
    params.rest_density = 1.0F - ( 4.0F * max_speed * max_speed / ( sound_speed * sound_speed ) );

    return params;
}

ParticleSystem::ParticleSystem( vlk::objs::VulkanGpu& gpu )
    : gpu_( gpu )
{
}

auto ParticleSystem::initialize(
    ParticleSystemSettings settings,
    vlk::CommandPool&      command_pool,
    vk::Queue const&       queue
) -> utils::Result< void >
{
    if ( this->is_initialized( ) )
    {
        return utils::success( );
    }
    LTB_CHECK_VALID( gpu_.is_initialized( ) );
    LTB_CHECK_VALID( settings.fill.particle_count > 0U );
    LTB_CHECK_VALID( settings.fill.packing_fraction > 0.0F );
    LTB_CHECK_VALID( settings.fill.packing_fraction < 0.75F );

    settings_ = std::move( settings );
    radius_   = particle_radius( settings_.tunnel, settings_.fill );

    auto const field = build_distance_field( settings_.solids, settings_.tunnel, settings_.solid_field_size );

    // The area of the tunnel that is not inside a solid.
    auto const free_texels = std::ranges::count_if( field.texels, []( glm::vec4 const& texel ) {
        return texel.x > 0.0F;
    } );
    auto const free_area = settings_.tunnel.size.x * settings_.tunnel.size.y
                         * static_cast< float32 >( free_texels )
                         / static_cast< float32 >( field.texels.size( ) );
    auto const area_per_particle = free_area / static_cast< float32 >( settings_.fill.particle_count );

    spacing_ = std::sqrt( area_per_particle );

    // SPH measures density by adding up a smooth kernel centered on each nearby particle.
    // Choose the mass that makes evenly spaced particles add up to a density of exactly 1
    // (the sum over a grid of particles is close to, but not exactly, 1 / spacing^2).
    {
        auto const h = 0.5F * settings_.sph_support_scale * spacing_;
        auto const kernel = [ h ]( float32 const dist ) {
            auto const q = dist / h;
            if ( q >= 2.0F )
            {
                return 0.0F;
            }
            // Matches `kernel_scale( ) * kernel_shape( )` in `sph.glsl`.
            auto const t = 1.0F - ( 0.5F * q );
            return ( 7.0F / ( 4.0F * glm::pi< float32 >( ) * h * h ) ) * ( t * t * t * t )
                 * ( ( 2.0F * q ) + 1.0F );
        };

        auto const reach      = static_cast< int32 >( std::ceil( settings_.sph_support_scale ) );
        auto       kernel_sum = 0.0F;
        for ( auto y = -reach; y <= reach; ++y )
        {
            for ( auto x = -reach; x <= reach; ++x )
            {
                kernel_sum += kernel( spacing_ * glm::length( glm::vec2( glm::ivec2{ x, y } ) ) );
            }
        }
        particle_mass_ = 1.0F / kernel_sum;
    }

    // Cells are at least as wide as the distance within which particles interact so
    // a particle can only interact with particles in its own or adjacent cells.
    auto interaction_distance = 2.0F * radius_;
    if ( FluidModel::Sph == settings_.model )
    {
        LTB_CHECK_VALID( settings_.sph_support_scale >= 2.0F );
        interaction_distance = std::max( interaction_distance, settings_.sph_support_scale * spacing_ );
    }
    grid_size_ = glm::max( glm::ivec2( 1 ), glm::ivec2( settings_.tunnel.size / interaction_distance ) );

    LTB_CHECK_VALID( settings_.field_cell_scale > 0U );
    field_scale_ = settings_.field_cell_scale;

    if ( FluidModel::Sph == settings_.model )
    {
        // SPH cells are subdivided so the cells searched for neighbors hug the kernel's
        // circular support more closely (fewer particles out of range are checked). The
        // field cells keep covering whole undivided cells.
        grid_size_ *= sph_cell_subdivision;
        field_scale_ *= static_cast< uint32 >( sph_cell_subdivision );

        // The wrapped columns at the ends of the tunnel must not overlap.
        LTB_CHECK_VALID( grid_size_.x > 2 * sph_cell_subdivision );
    }

    auto const field_scale = static_cast< int32 >( field_scale_ );
    field_size_            = ( grid_size_ + ( field_scale - 1 ) ) / field_scale;

    auto offset = 0U;
    auto count  = static_cast< uint32 >( grid_size_.x ) * static_cast< uint32 >( grid_size_.y );
    for ( auto& level : scan_levels_ )
    {
        level  = ScanLevel{ .offset = offset, .count = count };
        offset += count;
        count  = group_count( count, scan_block_size );
    }
    // The last level is the single (unused) total of the level before it.
    LTB_CHECK_VALID( 1U == scan_levels_.back( ).count );

    // Particles start outside of the solids.
    auto const solid_distance = [ & ]( glm::vec2 const& point ) {
        return sample_distance( field, settings_.tunnel, point );
    };
    LTB_CHECK(
        this->initialize_buffers(
            fill_tunnel( settings_.tunnel, settings_.fill, solid_distance ),
            field,
            command_pool,
            queue
        )
    );

    // The prefix sum uses subgroup arithmetic (core since Vulkan 1.1, but optional).
    {
        auto const properties = gpu_.physical_device( )
                                    .get( )
                                    .getProperties2<
                                        vk::PhysicalDeviceProperties2,
                                        vk::PhysicalDeviceSubgroupProperties >( )
                                    .get< vk::PhysicalDeviceSubgroupProperties >( );

        LTB_CHECK_VALID( properties.supportedStages & vk::ShaderStageFlagBits::eCompute );
        LTB_CHECK_VALID(
            properties.supportedOperations & vk::SubgroupFeatureFlagBits::eArithmetic
        );
    }

    LTB_CHECK( this->initialize_pipeline( count_pipeline_, "grid_count.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( scan_up_pipeline_, "grid_scan_up.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( scan_down_pipeline_, "grid_scan_down.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( scatter_pipeline_, "grid_scatter.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( collide_pipeline_, "collide_integrate.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( fields_pipeline_, "fields.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( sph_density_pipeline_, "sph_density.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( sph_integrate_pipeline_, "sph_integrate.comp.spv" ) );

    initialized_ = true;

    return utils::success( );
}

auto ParticleSystem::is_initialized( ) const -> bool
{
    return initialized_;
}

auto ParticleSystem::initialize_buffers(
    std::vector< Particle > const& particles,
    DistanceField const&           field,
    vlk::CommandPool&              command_pool,
    vk::Queue const&               queue
) -> utils::Result< void >
{
    auto const particles_size = particles.size( ) * sizeof( Particle );

    auto particles_layout = vlk::MemoryLayout{ };
    vlk::append_memory_size_n( particles_layout, particles_size, 2U );

    LTB_CHECK( particles_.initialize( {
        .layout       = std::move( particles_layout ),
        .buffer_usage = vk::BufferUsageFlagBits::eStorageBuffer
                      | vk::BufferUsageFlagBits::eVertexBuffer
                      | vk::BufferUsageFlagBits::eTransferSrc
                      | vk::BufferUsageFlagBits::eTransferDst,
        .memory_properties = vk::MemoryPropertyFlagBits::eDeviceLocal,
    } ) );

    auto const grid_size = ( scan_levels_.back( ).offset + scan_levels_.back( ).count )
                         * sizeof( uint32 );

    LTB_CHECK( grid_.initialize( {
        .layout       = { .total_size = grid_size },
        .buffer_usage = vk::BufferUsageFlagBits::eStorageBuffer
                      | vk::BufferUsageFlagBits::eTransferDst,
        .memory_properties = vk::MemoryPropertyFlagBits::eDeviceLocal,
    } ) );

    LTB_CHECK( densities_.initialize( {
        .layout       = { .total_size = particles.size( ) * sizeof( float32 ) },
        .buffer_usage = vk::BufferUsageFlagBits::eStorageBuffer,
        .memory_properties = vk::MemoryPropertyFlagBits::eDeviceLocal,
    } ) );

    // The flow fields start at zero.
    {
        auto const fields_size = static_cast< vk::DeviceSize >( field_size_.x )
                               * static_cast< vk::DeviceSize >( field_size_.y ) * sizeof( glm::vec4 );

        LTB_CHECK( fields_.initialize( {
            .layout       = { .total_size = fields_size },
            .buffer_usage = vk::BufferUsageFlagBits::eStorageBuffer
                          | vk::BufferUsageFlagBits::eTransferSrc
                          | vk::BufferUsageFlagBits::eTransferDst,
            .memory_properties = vk::MemoryPropertyFlagBits::eDeviceLocal,
        } ) );

        auto clear_command_buffer = vlk::CommandBuffer{ gpu_.device( ), command_pool };
        LTB_CHECK( clear_command_buffer.initialize( ) );

        VK_CHECK( clear_command_buffer.get( ).begin(
            vk::CommandBufferBeginInfo{ }.setFlags( vk::CommandBufferUsageFlagBits::eOneTimeSubmit )
        ) );
        clear_command_buffer.get( ).fillBuffer( fields_.buffer( ).get( ), 0U, fields_size, 0U );
        VK_CHECK( clear_command_buffer.get( ).end( ) );

        VK_CHECK( queue.submit( vk::SubmitInfo{ }.setCommandBuffers( clear_command_buffer.get( ) ) ) );
        VK_CHECK( queue.waitIdle( ) );
    }

    // Upload the distance field of the solids.
    {
        auto const field_size = field.texels.size( ) * sizeof( glm::vec4 );

        LTB_CHECK( solids_.initialize( {
            .layout       = { .total_size = field_size },
            .buffer_usage = vk::BufferUsageFlagBits::eStorageBuffer
                          | vk::BufferUsageFlagBits::eTransferDst,
            .memory_properties = vk::MemoryPropertyFlagBits::eDeviceLocal,
        } ) );

        auto field_staging = vlk::objs::VulkanBuffer{ gpu_ };

        LTB_CHECK( field_staging.initialize( {
            .layout       = { .total_size = field_size },
            .buffer_usage = vk::BufferUsageFlagBits::eTransferSrc,
            .memory_properties
            = vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
        } ) );

        LTB_CHECK( auto* const field_data, field_staging.memory( ).map( ) );
        std::memcpy( field_data, field.texels.data( ), field_size );
        field_staging.memory( ).unmap( );

        LTB_CHECK(
            vlk::copy_buffer(
                gpu_.device( ),
                command_pool,
                queue,
                field_staging.buffer( ),
                solids_.buffer( ),
                { vk::BufferCopy{ }.setSize( field_size ) }
            )
        );
    }

    // Upload the initial particles.
    auto staging = vlk::objs::VulkanBuffer{ gpu_ };

    LTB_CHECK( staging.initialize( {
        .layout       = { .total_size = particles_size },
        .buffer_usage = vk::BufferUsageFlagBits::eTransferSrc,
        .memory_properties
        = vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
    } ) );

    LTB_CHECK( auto* const staging_data, staging.memory( ).map( ) );
    std::memcpy( staging_data, particles.data( ), particles_size );
    staging.memory( ).unmap( );

    auto const& range_a = particles_.layout( ).ranges[ particles_a_range ];

    return vlk::copy_buffer(
        gpu_.device( ),
        command_pool,
        queue,
        staging.buffer( ),
        particles_.buffer( ),
        { vk::BufferCopy{ }.setSrcOffset( 0U ).setDstOffset( range_a.offset ).setSize( particles_size ) }
    );
}

auto ParticleSystem::initialize_pipeline(
    vlk::objs::VulkanComputePipeline& pipeline,
    std::string_view const            shader_name
) -> utils::Result< void >
{
    constexpr auto binding_count = 6U;

    auto bindings = std::vector< vk::DescriptorSetLayoutBinding >{ };
    for ( auto binding = 0U; binding < binding_count; ++binding )
    {
        bindings.push_back(
            vk::DescriptorSetLayoutBinding{ }
                .setBinding( binding )
                .setDescriptorType( vk::DescriptorType::eStorageBuffer )
                .setDescriptorCount( 1U )
                .setStageFlags( vk::ShaderStageFlagBits::eCompute )
        );
    }

    LTB_CHECK( pipeline.initialize( {
        .shader_module = {
            .spirv_file = config::shader_dir_path( ) / shader_name,
            .stage      = vk::ShaderStageFlagBits::eCompute,
        },
        .descriptor_set_count   = 1U,
        .uniform_bindings       = std::move( bindings ),
        .uniform_push_constants = {
            vk::PushConstantRange{ }
                .setStageFlags( vk::ShaderStageFlagBits::eCompute )
                .setOffset( 0U )
                .setSize( sizeof( StepConstants ) ),
        },
    } ) );

    auto const& range_a = particles_.layout( ).ranges[ particles_a_range ];
    auto const& range_b = particles_.layout( ).ranges[ particles_b_range ];

    auto const buffer_infos = std::array{
        vk::DescriptorBufferInfo{ }
            .setBuffer( particles_.buffer( ).get( ) )
            .setOffset( range_a.offset )
            .setRange( range_a.size ),
        vk::DescriptorBufferInfo{ }
            .setBuffer( particles_.buffer( ).get( ) )
            .setOffset( range_b.offset )
            .setRange( range_b.size ),
        vk::DescriptorBufferInfo{ }
            .setBuffer( grid_.buffer( ).get( ) )
            .setOffset( 0U )
            .setRange( grid_.layout( ).total_size ),
        vk::DescriptorBufferInfo{ }
            .setBuffer( solids_.buffer( ).get( ) )
            .setOffset( 0U )
            .setRange( solids_.layout( ).total_size ),
        vk::DescriptorBufferInfo{ }
            .setBuffer( fields_.buffer( ).get( ) )
            .setOffset( 0U )
            .setRange( fields_.layout( ).total_size ),
        vk::DescriptorBufferInfo{ }
            .setBuffer( densities_.buffer( ).get( ) )
            .setOffset( 0U )
            .setRange( densities_.layout( ).total_size ),
    };

    auto const& descriptor_sets = pipeline.descriptor_sets( ).get( );
    LTB_CHECK_VALID( 1UZ == descriptor_sets.size( ) );

    auto descriptor_writes = std::vector< vk::WriteDescriptorSet >{ };
    for ( auto binding = 0U; binding < binding_count; ++binding )
    {
        descriptor_writes.push_back(
            vk::WriteDescriptorSet{ }
                .setDstSet( descriptor_sets.front( ) )
                .setDstBinding( binding )
                .setDstArrayElement( 0U )
                .setDescriptorType( vk::DescriptorType::eStorageBuffer )
                .setBufferInfo( buffer_infos[ binding ] )
        );
    }
    gpu_.device( ).get( ).updateDescriptorSets( descriptor_writes, { } );

    return utils::success( );
}

auto ParticleSystem::record_step( vk::CommandBuffer const& command_buffer, StepParams const& params )
    -> void
{
    auto constants = StepConstants{
        .tunnel_size    = settings_.tunnel.size,
        .delta_time     = params.delta_time,
        .radius         = radius_,
        .grid_size      = grid_size_,
        .particle_count = settings_.fill.particle_count,
        .stiffness      = params.stiffness,
        .viscosity      = params.viscosity,
        .damping        = params.damping,

        .step_index = step_index_++,
        .solid_size = settings_.solid_field_size,

        .flow_speed    = params.flow_speed,
        .thermal_speed = params.thermal_speed,
        .inflow_width  = params.buffers ? params.buffer_width : params.inflow_width,
        .inflow_rate   = params.inflow_rate,

        .wall_friction    = params.wall_friction,
        .wall_restitution = params.wall_restitution,

        .field_size  = field_size_,
        .field_scale = field_scale_,
        .field_blend = params.field_blend,

        .boundary_flags = ( ( params.respawn || params.buffers ) ? respawn_flag : 0U )
                        | ( params.buffers ? buffers_flag : 0U ),

        .model            = static_cast< uint32 >( settings_.model ),
        .sound_speed      = params.sound_speed,
        .smoothing_length = this->smoothing_length( ),
        .particle_mass    = particle_mass_,
        .rest_density     = params.rest_density,
    };

    // Binds a pipeline with the current constants and runs it over `item_count` items.
    auto const dispatch = [ & ]( vlk::objs::VulkanComputePipeline& pipeline,
                                 uint32 const                      item_count,
                                 uint32 const                      local_size ) {
        auto const& layout = pipeline.pipeline_layout( ).get( );

        pipeline.bind( command_buffer );
        command_buffer.bindDescriptorSets(
            vk::PipelineBindPoint::eCompute,
            layout,
            0U,
            pipeline.descriptor_sets( ).get( ).front( ),
            { }
        );
        command_buffer.pushConstants< StepConstants >(
            layout,
            vk::ShaderStageFlagBits::eCompute,
            0U,
            constants
        );
        command_buffer.dispatch( group_count( item_count, local_size ), 1U, 1U );

        full_barrier( command_buffer );
    };

    // Profiling: timestamps after each pass (every pass ends with a full barrier).
    auto const mark = [ & ]( char const* const name ) {
        if ( query_pool_ )
        {
            command_buffer.writeTimestamp(
                vk::PipelineStageFlagBits::eBottomOfPipe,
                query_pool_.get( ),
                static_cast< uint32 >( pass_names_.size( ) + 1UZ )
            );
            pass_names_.push_back( name );
        }
    };

    auto const particle_count = settings_.fill.particle_count;

    full_barrier( command_buffer );

    if ( query_pool_ )
    {
        pass_names_.clear( );
        command_buffer.resetQueryPool( query_pool_.get( ), 0U, max_timestamps );
        command_buffer
            .writeTimestamp( vk::PipelineStageFlagBits::eBottomOfPipe, query_pool_.get( ), 0U );
    }

    // 1. Clear the cell counters.
    command_buffer.fillBuffer(
        grid_.buffer( ).get( ),
        0U,
        scan_levels_.front( ).count * sizeof( uint32 ),
        0U
    );
    full_barrier( command_buffer );
    mark( "clear" );

    // 2. Count the particles in each cell.
    dispatch( count_pipeline_, particle_count, 256U );
    mark( "count" );

    // 3. Prefix sum. Sweep up the hierarchy summing blocks, then back down adding offsets.
    for ( auto level = 0UZ; ( level + 1UZ ) < scan_levels_.size( ); ++level )
    {
        constants.scan_child_offset  = scan_levels_[ level ].offset;
        constants.scan_child_count   = scan_levels_[ level ].count;
        constants.scan_parent_offset = scan_levels_[ level + 1UZ ].offset;

        // One workgroup per block (the parent level has one element per block).
        dispatch(
            scan_up_pipeline_,
            scan_levels_[ level + 1UZ ].count * scan_threads,
            scan_threads
        );
    }
    for ( auto level = scan_levels_.size( ) - 2UZ; level-- > 0UZ; )
    {
        constants.scan_child_offset  = scan_levels_[ level ].offset;
        constants.scan_child_count   = scan_levels_[ level ].count;
        constants.scan_parent_offset = scan_levels_[ level + 1UZ ].offset;

        dispatch( scan_down_pipeline_, scan_levels_[ level ].count, 256U );
    }
    mark( "scan" );

    // 4. Sort the particles by cell.
    dispatch( scatter_pipeline_, particle_count, 256U );
    mark( "scatter" );

    auto const field_count = static_cast< uint32 >( field_size_.x )
                           * static_cast< uint32 >( field_size_.y );

    if ( FluidModel::Sph == settings_.model )
    {
        // 5. The density of the fluid at each particle, then the flow
        //    fields (which include the pressure computed from the density).
        dispatch( sph_density_pipeline_, particle_count, 256U );
        mark( "sph_density" );
        dispatch( fields_pipeline_, field_count, 64U );
        mark( "fields" );

        // 6. Pressure and viscous forces, then integrate.
        dispatch( sph_integrate_pipeline_, particle_count, 256U );
        mark( "sph_integrate" );
    }
    else
    {
        // Measure the flow fields from the sorted particles.
        dispatch( fields_pipeline_, field_count, 64U );
        mark( "fields" );

        // 5. Collide and integrate.
        dispatch( collide_pipeline_, particle_count, 256U );
        mark( "collide_integrate" );
    }
}

auto ParticleSystem::enable_profiling( ) -> utils::Result< void >
{
    LTB_CHECK_VALID( this->is_initialized( ) );

    auto const& limits = gpu_.physical_device( ).properties( ).limits;
    if ( ( VK_FALSE == limits.timestampComputeAndGraphics ) || ( limits.timestampPeriod <= 0.0F ) )
    {
        return LTB_MAKE_UNEXPECTED_ERROR( "The GPU does not support compute timestamps" );
    }

    VK_CHECK(
        query_pool_,
        gpu_.device( ).get( ).createQueryPoolUnique(
            vk::QueryPoolCreateInfo{ }
                .setQueryType( vk::QueryType::eTimestamp )
                .setQueryCount( max_timestamps )
        )
    );
    return utils::success( );
}

auto ParticleSystem::read_profile( ) -> utils::Result< std::vector< PassTiming > >
{
    LTB_CHECK_VALID( query_pool_ );
    LTB_CHECK_VALID( !pass_names_.empty( ) );

    VK_CHECK( gpu_.device( ).get( ).waitIdle( ) );

    auto timestamps = std::vector< uint64 >( pass_names_.size( ) + 1UZ );

    VK_CHECK( gpu_.device( ).get( ).getQueryPoolResults(
        query_pool_.get( ),
        0U,
        static_cast< uint32 >( timestamps.size( ) ),
        timestamps.size( ) * sizeof( uint64 ),
        timestamps.data( ),
        sizeof( uint64 ),
        vk::QueryResultFlagBits::e64 | vk::QueryResultFlagBits::eWait
    ) );

    auto const nanoseconds_per_tick
        = static_cast< float64 >( gpu_.physical_device( ).properties( ).limits.timestampPeriod );

    auto timings = std::vector< PassTiming >{ };
    timings.reserve( pass_names_.size( ) );
    for ( auto i = 0UZ; i < pass_names_.size( ); ++i )
    {
        timings.push_back( {
            .name         = pass_names_[ i ],
            .milliseconds = static_cast< float64 >( timestamps[ i + 1UZ ] - timestamps[ i ] )
                          * nanoseconds_per_tick * 1.0e-6,
        } );
    }
    return timings;
}

auto ParticleSystem::read_particles( vlk::CommandPool& command_pool, vk::Queue const& queue )
    -> utils::Result< std::vector< Particle > >
{
    LTB_CHECK_VALID( this->is_initialized( ) );

    VK_CHECK( gpu_.device( ).get( ).waitIdle( ) );

    auto const& range_a = particles_.layout( ).ranges[ particles_a_range ];

    auto particles = std::vector< Particle >( settings_.fill.particle_count );
    auto const particles_size = particles.size( ) * sizeof( Particle );

    auto staging = vlk::objs::VulkanBuffer{ gpu_ };

    LTB_CHECK( staging.initialize( {
        .layout       = { .total_size = particles_size },
        .buffer_usage = vk::BufferUsageFlagBits::eTransferDst,
        .memory_properties
        = vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
    } ) );

    LTB_CHECK(
        vlk::copy_buffer(
            gpu_.device( ),
            command_pool,
            queue,
            particles_.buffer( ),
            staging.buffer( ),
            { vk::BufferCopy{ }.setSrcOffset( range_a.offset ).setDstOffset( 0U ).setSize( particles_size ) }
        )
    );

    LTB_CHECK( auto const* const staging_data, staging.memory( ).map( ) );
    std::memcpy( particles.data( ), staging_data, particles_size );
    staging.memory( ).unmap( );

    return particles;
}

auto ParticleSystem::particle_buffer( ) const -> vk::Buffer const&
{
    return particles_.buffer( ).get( );
}

auto ParticleSystem::solid_buffer( ) const -> vlk::objs::VulkanBuffer const&
{
    return solids_;
}

auto ParticleSystem::field_buffer( ) const -> vlk::objs::VulkanBuffer const&
{
    return fields_;
}

auto ParticleSystem::field_size( ) const -> glm::ivec2 const&
{
    return field_size_;
}

auto ParticleSystem::particle_count( ) const -> uint32
{
    return settings_.fill.particle_count;
}

auto ParticleSystem::radius( ) const -> float32
{
    return radius_;
}

auto ParticleSystem::spacing( ) const -> float32
{
    return spacing_;
}

auto ParticleSystem::smoothing_length( ) const -> float32
{
    return 0.5F * settings_.sph_support_scale * spacing_;
}

auto ParticleSystem::tunnel( ) const -> Tunnel const&
{
    return settings_.tunnel;
}

auto ParticleSystem::grid_size( ) const -> glm::ivec2 const&
{
    return grid_size_;
}

auto ParticleSystem::settings( ) const -> ParticleSystemSettings const&
{
    return settings_;
}

} // namespace ltb::flew
