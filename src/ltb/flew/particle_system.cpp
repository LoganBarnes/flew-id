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

// standard
#include <cstring>

namespace ltb::flew
{
namespace
{

/// \brief Matches `SCAN_BLOCK_SIZE` in `particle.glsl`.
constexpr auto scan_block_size = 256U;

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
};

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

    // Cells are at least one particle diameter wide so a particle
    // can only touch particles in its own or adjacent cells.
    grid_size_ = glm::max( glm::ivec2( 1 ), glm::ivec2( settings_.tunnel.size / ( 2.0F * radius_ ) ) );

    LTB_CHECK_VALID( settings_.field_cell_scale > 0U );
    auto const field_scale = static_cast< int32 >( settings_.field_cell_scale );
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

    auto const field = build_distance_field( settings_.solids, settings_.tunnel, settings_.solid_field_size );

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

    LTB_CHECK( this->initialize_pipeline( count_pipeline_, "grid_count.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( scan_up_pipeline_, "grid_scan_up.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( scan_down_pipeline_, "grid_scan_down.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( scatter_pipeline_, "grid_scatter.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( collide_pipeline_, "collide_integrate.comp.spv" ) );
    LTB_CHECK( this->initialize_pipeline( fields_pipeline_, "fields.comp.spv" ) );

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
    constexpr auto binding_count = 5U;

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
        .inflow_width  = params.inflow_width,
        .inflow_rate   = params.inflow_rate,

        .wall_friction    = params.wall_friction,
        .wall_restitution = params.wall_restitution,

        .field_size  = field_size_,
        .field_scale = settings_.field_cell_scale,
        .field_blend = params.field_blend,
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

    auto const particle_count = settings_.fill.particle_count;

    // 1. Clear the cell counters.
    full_barrier( command_buffer );
    command_buffer.fillBuffer(
        grid_.buffer( ).get( ),
        0U,
        scan_levels_.front( ).count * sizeof( uint32 ),
        0U
    );
    full_barrier( command_buffer );

    // 2. Count the particles in each cell.
    dispatch( count_pipeline_, particle_count, 256U );

    // 3. Prefix sum. Sweep up the hierarchy summing blocks, then back down adding offsets.
    for ( auto level = 0UZ; ( level + 1UZ ) < scan_levels_.size( ); ++level )
    {
        constants.scan_child_offset  = scan_levels_[ level ].offset;
        constants.scan_child_count   = scan_levels_[ level ].count;
        constants.scan_parent_offset = scan_levels_[ level + 1UZ ].offset;

        // One invocation per block (which is the size of the parent level).
        dispatch( scan_up_pipeline_, scan_levels_[ level + 1UZ ].count, 64U );
    }
    for ( auto level = scan_levels_.size( ) - 2UZ; level-- > 0UZ; )
    {
        constants.scan_child_offset  = scan_levels_[ level ].offset;
        constants.scan_child_count   = scan_levels_[ level ].count;
        constants.scan_parent_offset = scan_levels_[ level + 1UZ ].offset;

        dispatch( scan_down_pipeline_, scan_levels_[ level ].count, 256U );
    }

    // 4. Sort the particles by cell.
    dispatch( scatter_pipeline_, particle_count, 256U );

    // Measure the flow fields from the sorted particles.
    dispatch(
        fields_pipeline_,
        static_cast< uint32 >( field_size_.x ) * static_cast< uint32 >( field_size_.y ),
        64U
    );

    // 5. Collide and integrate.
    dispatch( collide_pipeline_, particle_count, 256U );
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
