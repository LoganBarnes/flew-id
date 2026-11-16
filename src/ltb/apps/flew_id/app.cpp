// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#include "app.hpp"

// project
#include "ltb/exec/app_defaults.hpp"
#include "ltb/vlk/buffer_utils.hpp"
#include "ltb/vlk/check.hpp"
#include "ltb/flew/image_io.hpp"
#include "ltb/flew/ltb_flew_config.hpp"
#include "ltb/vlk/device_memory_utils.hpp"

// external
#include <spdlog/spdlog.h>

// standard
#include <cstdio>
#include <cstring>
#include <ranges>

namespace ltb::flew
{
namespace
{

/// \brief Matches `DisplayUbo` in `particles.vert`.
struct DisplayUniforms
{
    glm::mat4 clip_from_world = glm::identity< glm::mat4 >( );
    glm::vec2 viewport_size   = { };
    float32   diameter        = 0.0F;
    float32   max_speed       = 1.0F;
    glm::vec2 tunnel_size     = { };
    glm::ivec2 solid_size     = { };
    glm::ivec2 field_size     = { };

    int32   field_mode            = 0;
    float32 field_range           = 0.25F;
    float32 reference_density     = 1.0F;
    float32 reference_temperature = 1.0F;
    int32   particle_mode         = 0;
    float32 particle_brightness   = 1.0F;
    float32 buffer_width          = 0.0F;
    float32 dynamic_pressure      = 0.0F;
};

/// \brief The fraction of the tunnel covered by SPH particles. They are spread evenly
///        so this only determines how large the particles are drawn.
constexpr auto sph_packing_fraction = 0.6F;

constexpr auto field_mode_names = std::array{
    "None", "Speed", "Pressure", "Density", "Vorticity", "Temperature",
};
constexpr auto particle_mode_names = std::array{ "Speed", "Plain" };

/// \brief The number of simulation steps per update when running a check as fast as possible.
constexpr auto check_steps_per_update = 50U;

} // namespace

auto app_options( ) -> AppOptions&
{
    static auto options = AppOptions{ };
    return options;
}

FlewIdApp::FlewIdApp( window::GlfwContext& glfw_context, window::GlfwWindow& glfw_window )
    : glfw_context_( glfw_context )
    , glfw_window_( glfw_window )
{
}

auto FlewIdApp::initialize( ) -> utils::Result< exec::UpdateLoopStatus >
{
    if ( this->is_initialized( ) )
    {
        return exec::UpdateLoopStatus{ };
    }

    LTB_CHECK( this->initialize_gpu_presentation( )
                   .and_then( &FlewIdApp::initialize_simulation )
                   .and_then( &FlewIdApp::initialize_display_pipeline )
                   .and_then( &FlewIdApp::initialize_camera ) );

    if ( !app_options( ).video.empty( ) )
    {
        LTB_CHECK( this->start_recording( ) );
    }

    camera_.resize( glm::vec2( glfw_window_.framebuffer_size( ) ) );

    initialized_ = true;

    return exec::UpdateLoopStatus{ };
}

auto FlewIdApp::is_initialized( ) const -> bool
{
    return initialized_;
}

auto FlewIdApp::fixed_step_update( exec::UpdateLoopStatus const& status )
    -> exec::UpdateRequests
{
    auto requests = status.requests;

    // While recording, the simulation advances a fixed number of steps
    // per rendered frame (see frame_update) instead of following the clock.
    if ( finishing_ || this->is_recording( ) || ( paused_ && !single_step_ ) )
    {
        return requests;
    }
    single_step_ = false;

    return this->advance( requests );
}

auto FlewIdApp::advance( exec::UpdateRequests requests ) -> exec::UpdateRequests
{
    if ( auto result = this->compute( ); !result )
    {
        spdlog::error(
            "FlewIdApp::compute() failed:\n"
            "{}",
            result.error( ).debug_error_message( )
        );
        check_failed_             = true;
        requests.exit_update_loop = true;
    }

    auto const& options = app_options( );
    if ( options.max_steps > 0U && step_count_ >= options.max_steps )
    {
        // Render one more frame (so it can be captured) and then exit.
        finishing_ = true;
    }
    app_options( ).failed = check_failed_;

    return requests;
}

auto FlewIdApp::frame_update( exec::UpdateLoopStatus const& status ) -> exec::UpdateRequests
{
    if ( restart_requested_ )
    {
        restart_requested_ = false;

        if ( auto result = this->start_scenario( ).and_then( [ this ] {
                 return this->update_solid_descriptors( );
             } );
             !result )
        {
            spdlog::error( "Failed to restart:\n{}", result.error( ).debug_error_message( ) );
        }
    }

    auto requests = status.requests;

    if ( this->is_recording( ) && !finishing_ && !paused_ )
    {
        requests = this->advance( requests );
    }

    imgui_.new_frame( );
    if ( !app_options( ).hide_gui )
    {
        this->configure_gui( );
    }

    if ( auto result = this->render( ); !result )
    {
        spdlog::error(
            "FlewIdApp::render() failed:\n"
            "{}",
            result.error( ).debug_error_message( )
        );
    }

    if ( finishing_ )
    {
        if ( auto result = this->finish( ); !result )
        {
            spdlog::error( "{}", result.error( ).debug_error_message( ) );
            check_failed_ = true;
        }
        app_options( ).failed     = check_failed_;
        requests.exit_update_loop = true;
    }

    return requests;
}

auto FlewIdApp::configure_gui( ) -> void
{
    if ( camera_.handle_inputs( ) )
    {
        camera_frames_updated_.clear( );
    }

    ImGui::DockSpaceOverViewport( 0, nullptr, ImGuiDockNodeFlags_PassthruCentralNode );

    if ( ImGui::Begin( "Info" ) )
    {
        ImGui::Text( "Particles: %u", particles_->particle_count( ) );
        ImGui::Text( "Radius: %.5f", static_cast< float64 >( particles_->radius( ) ) );
        ImGui::Text( "Grid: %d x %d", particles_->grid_size( ).x, particles_->grid_size( ).y );
        ImGui::Text( "Steps: %llu", static_cast< unsigned long long >( step_count_ ) );
        ImGui::Text( "FPS: %.1f", static_cast< float64 >( ImGui::GetIO( ).Framerate ) );

        ImGui::SeparatorText( "Scenario" );
        auto scenario_index = static_cast< int32 >( scenario_ );
        if ( ImGui::Combo(
                 "Scenario",
                 &scenario_index,
                 scenario_names.data( ),
                 static_cast< int32 >( scenario_names.size( ) )
             ) )
        {
            scenario_          = static_cast< Scenario >( scenario_index );
            scenario_settings_ = default_scenario_settings( scenario_ );
            restart_requested_ = true;
        }
        auto model_index = static_cast< int32 >( model_ );
        if ( ImGui::Combo(
                 "Fluid model",
                 &model_index,
                 fluid_model_names.data( ),
                 static_cast< int32 >( fluid_model_names.size( ) )
             ) )
        {
            model_             = static_cast< FluidModel >( model_index );
            show_particles_    = ( FluidModel::Gas == model_ );
            restart_requested_ = true;
        }
        ImGui::SetItemTooltip(
            "Gas: colliding disks with random motion (noisy, compressible).\n"
            "SPH: smooth parcels of fluid with a chosen viscosity."
        );
        ImGui::SliderFloat( "Angle of attack", &scenario_settings_.angle_degrees, -10.0F, 45.0F );
        ImGui::SliderFloat( "Flow speed", &scenario_settings_.flow_speed, 0.05F, 2.0F );
        ImGui::SliderFloat( "Venturi throat", &scenario_settings_.throat_fraction, 0.2F, 0.9F );
        auto particle_count = static_cast< int32 >( particle_count_ );
        if ( ImGui::InputInt( "Particles", &particle_count, 50'000, 500'000 ) )
        {
            particle_count_ = static_cast< uint32 >( std::clamp( particle_count, 1'000, 8'000'000 ) );
        }
        if ( ImGui::Button( "Restart" ) )
        {
            restart_requested_ = true;
        }

        ImGui::SeparatorText( "Simulation" );
        ImGui::Checkbox( "Paused", &paused_ );
        ImGui::SameLine( );
        if ( ImGui::Button( "Step" ) )
        {
            single_step_ = true;
        }
        ImGui::SliderInt( "Steps per update", &substeps_, 1, 200 );
        ImGui::SliderFloat(
            "Time step",
            &step_params_.delta_time,
            1.0e-5F,
            5.0e-3F,
            "%.5f",
            ImGuiSliderFlags_Logarithmic
        );
        if ( FluidModel::Sph == model_ )
        {
            ImGui::SliderFloat(
                "Viscosity",
                &step_params_.viscosity,
                1.0e-5F,
                1.0e-1F,
                "%.5f",
                ImGuiSliderFlags_Logarithmic
            );
            ImGui::SetItemTooltip(
                "Kinematic viscosity. The Reynolds number of the flow is\n"
                "flow speed x obstacle size / viscosity."
            );
            ImGui::SliderFloat( "Sound speed", &step_params_.sound_speed, 0.5F, 50.0F );
            ImGui::SetItemTooltip(
                "How strongly the fluid resists compression. Around ten times the\n"
                "flow speed behaves as incompressible. Higher values need a smaller time step."
            );
        }
        else
        {
            ImGui::SliderFloat(
                "Stiffness",
                &step_params_.stiffness,
                1.0e2F,
                1.0e8F,
                "%.0f",
                ImGuiSliderFlags_Logarithmic
            );
            ImGui::SliderFloat( "Damping", &step_params_.damping, 0.0F, 500.0F );
            ImGui::SliderFloat( "Viscosity", &step_params_.viscosity, 0.0F, 500.0F );
        }
        if ( ImGui::Checkbox( "Buffer zones", &step_params_.buffers ) )
        {
            camera_frames_updated_.clear( );
        }
        ImGui::SetItemTooltip(
            "Reset particles to the undisturbed flow in zones at both ends of the tunnel\n"
            "(like the settling chamber and diffuser of a real wind tunnel)."
        );
        if ( ImGui::SliderFloat( "Buffer length", &step_params_.buffer_width, 0.05F, 1.0F ) )
        {
            camera_frames_updated_.clear( );
        }
        ImGui::BeginDisabled( step_params_.buffers );
        ImGui::Checkbox( "Respawn at tunnel ends", &step_params_.respawn );
        ImGui::EndDisabled( );
        ImGui::SetItemTooltip(
            "Replace particles leaving either end of the tunnel with new particles of\n"
            "the incoming flow instead of carrying their velocity around to the other end."
        );
        ImGui::SliderFloat( "Wall friction", &step_params_.wall_friction, 0.0F, 1.0F );
        ImGui::SliderFloat( "Wall restitution", &step_params_.wall_restitution, 0.0F, 1.0F );

        ImGui::SeparatorText( "Display" );
        auto display_changed = false;

        display_changed |= ImGui::Combo(
            "Background",
            &field_mode_,
            field_mode_names.data( ),
            static_cast< int32 >( field_mode_names.size( ) )
        );
        display_changed |= ImGui::SliderFloat( "Field range", &field_range_, 0.01F, 2.0F );
        display_changed |= ImGui::SliderFloat( "Max speed", &max_display_speed_, 0.1F, 5.0F );
        ImGui::SliderFloat(
            "Field smoothing",
            &step_params_.field_blend,
            0.0001F,
            1.0F,
            "%.4f",
            ImGuiSliderFlags_Logarithmic
        );

        display_changed |= ImGui::Checkbox( "Show particles", &show_particles_ );
        display_changed |= ImGui::Combo(
            "Particle color",
            &particle_mode_,
            particle_mode_names.data( ),
            static_cast< int32 >( particle_mode_names.size( ) )
        );
        display_changed |= ImGui::SliderFloat( "Particle brightness", &particle_brightness_, 0.0F, 1.0F );

        if ( display_changed )
        {
            camera_frames_updated_.clear( );
        }
    }
    ImGui::End( );
}

auto FlewIdApp::on_resize( glm::ivec2 const size ) -> utils::Result< void >
{
    camera_.resize( glm::vec2( size ) );
    camera_frames_updated_.clear( );

    return presentation_.rebuild( {
        .swapchain = presentation_.swapchain( ).settings( ),
    } );
}

auto FlewIdApp::clean_up( ) -> utils::Result< void >
{
    this->stop_recording( );

    VK_CHECK( gpu_.device( ).get( ).waitIdle( ) );

    return utils::success( );
}

auto FlewIdApp::initialize_gpu_presentation( ) -> utils::Result< FlewIdApp* >
{
    auto gpu_settings = vlk::objs::VulkanGpuSettings{ };
    // Particles are drawn as points larger than one pixel.
    gpu_settings.device.device_features.push_back( &vk::PhysicalDeviceFeatures::largePoints );

    LTB_CHECK( gpu_.initialize( std::move( gpu_settings ) ) );

    LTB_CHECK_VALID( gpu_.device( ).queues( ).contains( vlk::QueueType::Graphics ) );
    auto const& graphics_queue = gpu_.device( ).queues( ).at( vlk::QueueType::Graphics );

    LTB_CHECK_VALID( gpu_.device( ).queues( ).contains( vlk::QueueType::Compute ) );
    auto const& compute_queue = gpu_.device( ).queues( ).at( vlk::QueueType::Compute );

    LTB_CHECK_VALID( graphics_queue == compute_queue );
    graphics_and_compute_queue_ = graphics_queue;

    LTB_CHECK_VALID( gpu_.device( ).queues( ).contains( vlk::QueueType::Surface ) );
    present_queue_ = gpu_.device( ).queues( ).at( vlk::QueueType::Surface );

    auto swapchain_settings = vlk::SwapchainSettings{
        .preferred_present_modes = {
            vk::PresentModeKHR::eImmediate,
            vk::PresentModeKHR::eFifoLatestReady,
            vk::PresentModeKHR::eFifo,
        },
        // Rendered images are copied when writing screenshots.
        .image_usage = vk::ImageUsageFlagBits::eColorAttachment
                     | vk::ImageUsageFlagBits::eTransferSrc,
    };
    auto render_pass_settings = vlk::render_pass_settings_3d(
        vk::Format::eUndefined,
        gpu_.physical_device( ).depth_image_format( )
    );

    LTB_CHECK( presentation_.initialize( {
        .swapchain   = std::move( swapchain_settings ),
        .render_pass = std::move( render_pass_settings ),
    } ) );

    LTB_CHECK( imgui_.initialize( ) );

    return this;
}

auto FlewIdApp::initialize_simulation( ) -> utils::Result< FlewIdApp* >
{
    LTB_CHECK( compute_cmd_and_sync_.initialize( {
        .frame_count  = exec::max_frames_in_flight,
        .image_count  = 0UZ,
        .command_pool = {
            .queue_type = vlk::QueueType::Compute,
        },
    } ) );

    auto const& options = app_options( );

    model_             = options.model;
    scenario_          = options.scenario;
    scenario_settings_ = default_scenario_settings( scenario_ );
    if ( options.angle_degrees )
    {
        scenario_settings_.angle_degrees = options.angle_degrees.value( );
    }
    if ( options.flow_speed )
    {
        scenario_settings_.flow_speed = options.flow_speed.value( );
    }
    if ( options.throat_fraction )
    {
        scenario_settings_.throat_fraction = options.throat_fraction.value( );
    }
    step_params_.buffers = !options.no_buffers;
    if ( options.particle_count > 0U )
    {
        particle_count_ = options.particle_count;
    }

    for ( auto mode = 0UZ; mode < field_mode_names.size( ); ++mode )
    {
        if ( options.field == field_mode_names[ mode ] )
        {
            field_mode_ = static_cast< int32 >( mode );
        }
    }
    // SPH particles are packed so tightly they only draw a moire pattern over the fields.
    show_particles_      = !options.hide_particles && ( FluidModel::Sph != model_ );
    step_params_.respawn = options.respawn;
    if ( options.steps_per_update > 0U )
    {
        substeps_ = static_cast< int32 >( options.steps_per_update );
    }

    LTB_CHECK( this->start_scenario( ) );

    if ( options.check )
    {
        LTB_CHECK(
            auto const initial_particles,
            particles_
                ->read_particles( compute_cmd_and_sync_.command_pool( ), graphics_and_compute_queue_ )
        );
        initial_stats_ = compute_stats( initial_particles, particles_->tunnel( ), particles_->radius( ) );
        if ( 0U == options.steps_per_update )
        {
            substeps_ = static_cast< int32 >( check_steps_per_update );
        }
    }

    return this;
}

auto FlewIdApp::start_scenario( ) -> utils::Result< void >
{
    // Nothing on the GPU can be using the previous particles when they are destroyed.
    VK_CHECK( gpu_.device( ).get( ).waitIdle( ) );

    auto settings   = ParticleSystemSettings{ };
    settings.tunnel = scenario_tunnel( scenario_ );
    // Keep the texels of the solids' distance field square.
    settings.solid_field_size.x = static_cast< int32 >(
        static_cast< float32 >( settings.solid_field_size.y ) * settings.tunnel.size.x
        / settings.tunnel.size.y
    );

    auto const sph = ( FluidModel::Sph == model_ );

    settings.model = model_;
    if ( sph )
    {
        // Parcels of fluid fill the tunnel evenly and have no random motion. The
        // flow fields are not noisy so they don't need to be averaged over large cells.
        settings.fill.packing_fraction = sph_packing_fraction;
        settings.field_cell_scale      = 1U;
    }
    thermal_speed_ = sph ? 0.0F : ParticleFillSettings{ }.thermal_speed;

    if ( auto const scale = app_options( ).field_cell_scale; scale > 0U )
    {
        settings.field_cell_scale = scale;
    }

    auto setup = make_scenario( scenario_, settings.tunnel, scenario_settings_ );

    settings.solids              = std::move( setup.solids );
    settings.fill.particle_count = particle_count_;
    settings.fill.thermal_speed  = thermal_speed_;
    settings.fill.mean_velocity  = { setup.flow_speed, 0.0F };

    auto particles = std::make_unique< ParticleSystem >( gpu_ );
    LTB_CHECK(
        particles->initialize(
            std::move( settings ),
            compute_cmd_and_sync_.command_pool( ),
            graphics_and_compute_queue_
        )
    );
    particles_ = std::move( particles );

    if ( sph || params_model_ != model_ )
    {
        // Start from the model's defaults, keeping the settings shared by both models.
        auto const previous = step_params_;

        step_params_ = sph ? default_sph_step_params( particles_->smoothing_length( ), setup.flow_speed )
                           : StepParams{ };

        step_params_.buffers      = previous.buffers;
        step_params_.buffer_width = previous.buffer_width;
        step_params_.respawn      = previous.respawn;
        if ( sph && params_model_ == model_ )
        {
            // Keep any adjustments made to the fluid.
            step_params_.viscosity     = previous.viscosity;
            step_params_.wall_friction = previous.wall_friction;
        }
        params_model_ = model_;
    }
    if ( !sph )
    {
        // Keep any adjustments made to the interaction settings.
        auto const defaults = default_step_params( particles_->radius( ), thermal_speed_ + setup.flow_speed );

        step_params_.delta_time    = defaults.delta_time;
        step_params_.stiffness     = defaults.stiffness;
        step_params_.flow_speed    = setup.flow_speed;
        step_params_.thermal_speed = thermal_speed_;
    }

    // Fit the whole tunnel in the window.
    auto const& tunnel = particles_->tunnel( );
    camera_.set_center( tunnel.size * 0.5F );
    camera_.set_width( tunnel.size.x * 1.1F );

    step_count_ = 0UZ;
    camera_frames_updated_.clear( );

    return utils::success( );
}

auto FlewIdApp::update_solid_descriptors( ) -> utils::Result< void >
{
    LTB_CHECK_VALID( background_.is_initialized( ) );
    LTB_CHECK_VALID( 1UZ == background_.descriptor_sets( ).size( ) );

    // Bindings 1 and 2 of the background pipeline.
    auto const buffers = std::array{ &particles_->solid_buffer( ), &particles_->field_buffer( ) };

    for ( auto index = 0U; index < buffers.size( ); ++index )
    {
        auto const buffer_info = vk::DescriptorBufferInfo{ }
                                     .setBuffer( buffers[ index ]->buffer( ).get( ) )
                                     .setOffset( 0U )
                                     .setRange( buffers[ index ]->layout( ).total_size );

        for ( auto const& descriptor_set : background_.descriptor_sets( ).front( ).get( ) )
        {
            auto const descriptor_write = vk::WriteDescriptorSet{ }
                                              .setDstSet( descriptor_set )
                                              .setDstBinding( index + 1U )
                                              .setDstArrayElement( 0U )
                                              .setDescriptorType( vk::DescriptorType::eStorageBuffer )
                                              .setBufferInfo( buffer_info );

            gpu_.device( ).get( ).updateDescriptorSets( descriptor_write, { } );
        }
    }

    return utils::success( );
}

auto FlewIdApp::initialize_display_pipeline( ) -> utils::Result< FlewIdApp* >
{
    auto shader_modules = std::vector{
        vlk::ShaderModuleSettings{
            .spirv_file = config::shader_dir_path( ) / "particles.vert.spv",
            .stage      = vk::ShaderStageFlagBits::eVertex,
        },
        vlk::ShaderModuleSettings{
            .spirv_file = config::shader_dir_path( ) / "particles.frag.spv",
            .stage      = vk::ShaderStageFlagBits::eFragment,
        },
    };

    auto uniform_bindings = std::vector{
        vk::DescriptorSetLayoutBinding{ }
            .setBinding( 0U )
            .setDescriptorType( vk::DescriptorType::eUniformBuffer )
            .setDescriptorCount( 1U )
            .setStageFlags( vk::ShaderStageFlagBits::eVertex ),
    };

    auto vertex_bindings = std::vector{
        vk::VertexInputBindingDescription{ }
            .setBinding( 0U )
            .setStride( sizeof( Particle ) )
            .setInputRate( vk::VertexInputRate::eVertex ),
    };

    auto vertex_attributes = std::vector{
        vk::VertexInputAttributeDescription{ }
            .setBinding( 0U )
            .setLocation( 0U )
            .setFormat( vk::Format::eR32G32Sfloat )
            .setOffset( offsetof( Particle, position ) ),
        vk::VertexInputAttributeDescription{ }
            .setBinding( 0U )
            .setLocation( 1U )
            .setFormat( vk::Format::eR32G32Sfloat )
            .setOffset( offsetof( Particle, velocity ) ),
    };

    LTB_CHECK( graphics_.initialize( {
        .shader_modules       = std::move( shader_modules ),
        .descriptor_set_count = exec::max_frames_in_flight,
        .uniform_binding_sets = { std::move( uniform_bindings ) },

        .pipeline = {
            .vertex_bindings    = std::move( vertex_bindings ),
            .vertex_attributes  = std::move( vertex_attributes ),
            .primitive_topology = vk::PrimitiveTopology::ePointList,
            // Everything is drawn back to front in a single plane.
            .depth_stencil  = vk::PipelineDepthStencilStateCreateInfo{ },
            .alpha_blending = true,
        },
    } ) );

    auto background_settings = vlk::objs::VulkanGraphicsPipelineSettings{
        .shader_modules = {
            {
                .spirv_file = config::shader_dir_path( ) / "field.vert.spv",
                .stage      = vk::ShaderStageFlagBits::eVertex,
            },
            {
                .spirv_file = config::shader_dir_path( ) / "field.frag.spv",
                .stage      = vk::ShaderStageFlagBits::eFragment,
            },
        },
        .descriptor_set_count = exec::max_frames_in_flight,
        .uniform_binding_sets = { {
            vk::DescriptorSetLayoutBinding{ }
                .setBinding( 0U )
                .setDescriptorType( vk::DescriptorType::eUniformBuffer )
                .setDescriptorCount( 1U )
                .setStageFlags( vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment ),
            vk::DescriptorSetLayoutBinding{ }
                .setBinding( 1U )
                .setDescriptorType( vk::DescriptorType::eStorageBuffer )
                .setDescriptorCount( 1U )
                .setStageFlags( vk::ShaderStageFlagBits::eFragment ),
            vk::DescriptorSetLayoutBinding{ }
                .setBinding( 2U )
                .setDescriptorType( vk::DescriptorType::eStorageBuffer )
                .setDescriptorCount( 1U )
                .setStageFlags( vk::ShaderStageFlagBits::eFragment ),
        } },
        .pipeline = {
            .primitive_topology = vk::PrimitiveTopology::eTriangleStrip,
            .depth_stencil      = vk::PipelineDepthStencilStateCreateInfo{ },
        },
    };
    background_settings.pipeline.rasterizer.setCullMode( vk::CullModeFlagBits::eNone );

    LTB_CHECK( background_.initialize( std::move( background_settings ) ) );

    LTB_CHECK( graphics_cmd_and_sync_.initialize( {
        .frame_count  = exec::max_frames_in_flight,
        .image_count  = static_cast< uint32 >( presentation_.swapchain( ).images( ).size( ) ),
        .command_pool = {
            .queue_type = vlk::QueueType::Graphics,
        },
    } ) );

    return this;
}

auto FlewIdApp::initialize_camera( ) -> utils::Result< FlewIdApp* >
{
    auto camera_buffer_layout = vlk::MemoryLayout{ };

    auto const ubo_alignment
        = gpu_.physical_device( ).properties( ).limits.minUniformBufferOffsetAlignment;

    append_memory_size_n(
        camera_buffer_layout,
        { sizeof( DisplayUniforms ), ubo_alignment },
        exec::max_frames_in_flight
    );

    LTB_CHECK( camera_ubo_.initialize( {
        .layout       = std::move( camera_buffer_layout ),
        .buffer_usage = vk::BufferUsageFlagBits::eUniformBuffer,
        .memory_properties
        = vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
        .store_mapped_value = true,
    } ) );

    LTB_CHECK_VALID( graphics_.is_initialized( ) );
    LTB_CHECK_VALID( 1UZ == graphics_.descriptor_sets( ).size( ) );
    LTB_CHECK_VALID( background_.is_initialized( ) );
    LTB_CHECK_VALID( 1UZ == background_.descriptor_sets( ).size( ) );

    // Both pipelines share the same display uniforms.
    for ( auto const* const pipeline : { &graphics_, &background_ } )
    for ( auto frame_index = 0U; frame_index < exec::max_frames_in_flight; ++frame_index )
    {
        auto const& graphics_descriptor_sets = pipeline->descriptor_sets( ).front( ).get( );

        LTB_CHECK_VALID( frame_index < camera_ubo_.layout( ).ranges.size( ) );
        LTB_CHECK_VALID( frame_index < graphics_descriptor_sets.size( ) );

        auto const& memory_range   = camera_ubo_.layout( ).ranges[ frame_index ];
        auto const& descriptor_set = graphics_descriptor_sets[ frame_index ];

        auto const descriptor_buffer_info = vk::DescriptorBufferInfo{ }
                                                .setBuffer( camera_ubo_.buffer( ).get( ) )
                                                .setOffset( memory_range.offset )
                                                .setRange( memory_range.size );

        auto const descriptor_writes = std::vector{
            vk::WriteDescriptorSet{ }
                .setDstSet( descriptor_set )
                .setDstBinding( 0U )
                .setDstArrayElement( 0U )
                .setDescriptorType( vk::DescriptorType::eUniformBuffer )
                .setBufferInfo( descriptor_buffer_info ),
        };

        gpu_.device( ).get( ).updateDescriptorSets( descriptor_writes, { } );
    }

    LTB_CHECK( this->update_solid_descriptors( ) );

    return this;
}

auto FlewIdApp::compute( ) -> utils::Result< void >
{
    LTB_CHECK(
        auto const maybe_frame,
        compute_cmd_and_sync_.start_frame( vlk::objs::ResetCommandBuffer::Yes )
    );

    if ( maybe_frame.has_value( ) )
    {
        auto const& frame = maybe_frame.value( );

        VK_CHECK( frame.command_buffer.begin( vk::CommandBufferBeginInfo{ } ) );
        for ( auto step = 0; step < substeps_; ++step )
        {
            particles_->record_step( frame.command_buffer, step_params_ );
        }
        VK_CHECK( frame.command_buffer.end( ) );

        step_count_ += static_cast< uint64 >( substeps_ );

        auto wait_until_signaled = std::vector< vlk::objs::SemaphoreAndStage >{ };

        if ( compute_semaphore_.has_value( ) )
        {
            wait_until_signaled.push_back( {
                .semaphore = compute_semaphore_.value( ),
                .stage     = vk::PipelineStageFlagBits::eComputeShader,
            } );
            compute_semaphore_ = std::nullopt;
        }

        auto const& compute_finished_semaphore
            = compute_cmd_and_sync_.get_frame_objects( ).frame_semaphore;

        LTB_CHECK( compute_cmd_and_sync_.end_frame(
            frame,
            std::move( wait_until_signaled ),
            { compute_finished_semaphore },
            graphics_and_compute_queue_
        ) );

        compute_semaphore_ = compute_finished_semaphore;
        compute_cmd_and_sync_.increment_frame( );
    }

    return utils::success( );
}

auto FlewIdApp::check_particles( ) -> utils::Result< void >
{
    LTB_CHECK(
        auto const particles,
        particles_
            ->read_particles( compute_cmd_and_sync_.command_pool( ), graphics_and_compute_queue_ )
    );
    auto const stats = compute_stats( particles, particles_->tunnel( ), particles_->radius( ) );

    spdlog::info( "Steps: {}", step_count_ );
    spdlog::info( "Initial: {}", to_string( initial_stats_ ) );
    spdlog::info( "Final  : {}", to_string( stats ) );

    // The gas conserves energy unless it is flowing. SPH is viscous so it never does.
    return check_stats(
        initial_stats_,
        stats,
        FluidModel::Sph == model_ || step_params_.flow_speed > 0.0F
    );
}

auto FlewIdApp::finish( ) -> utils::Result< void >
{
    auto const& options = app_options( );

    if ( !options.screenshot.empty( ) )
    {
        LTB_CHECK( this->write_screenshot( ) );
    }
    this->stop_recording( );
    if ( options.check )
    {
        LTB_CHECK( this->check_particles( ) );
    }
    return utils::success( );
}

auto FlewIdApp::is_recording( ) const -> bool
{
    return nullptr != video_pipe_;
}

auto FlewIdApp::start_recording( ) -> utils::Result< void >
{
    auto const& options = app_options( );
    auto const  extent  = presentation_.swapchain( ).settings( ).extent;

    auto const format = presentation_.swapchain( ).image_format( );
    LTB_CHECK_VALID( vk::Format::eB8G8R8A8Unorm == format || vk::Format::eB8G8R8A8Srgb == format );

    // ffmpeg encodes the raw frames written to its standard input.
    auto const command = fmt::format(
        "ffmpeg -loglevel error -y -f rawvideo -pix_fmt bgra -s {}x{} -r {} -i - "
        "-vf \"pad=ceil(iw/2)*2:ceil(ih/2)*2\" -c:v libx264 -pix_fmt yuv420p -crf 18 \"{}\"",
        extent.width,
        extent.height,
        options.video_fps,
        options.video.string( )
    );

    video_pipe_ = ::popen( command.c_str( ), "w" );
    LTB_CHECK_VALID( nullptr != video_pipe_ );
    video_extent_ = extent;

    spdlog::info( "Recording {} ({} steps per frame)", options.video.string( ), substeps_ );

    return utils::success( );
}

auto FlewIdApp::stop_recording( ) -> void
{
    if ( this->is_recording( ) )
    {
        ::pclose( video_pipe_ );
        video_pipe_ = nullptr;
        spdlog::info( "Wrote {} ({} frames)", app_options( ).video.string( ), video_frame_count_ );
    }
}

auto FlewIdApp::write_video_frame( ) -> utils::Result< void >
{
    auto const extent = presentation_.swapchain( ).settings( ).extent;
    if ( extent != video_extent_ )
    {
        spdlog::warn( "The window was resized. Recording stopped." );
        this->stop_recording( );
        return utils::success( );
    }

    LTB_CHECK( auto const pixels, this->read_capture( ) );
    LTB_CHECK_VALID( 1UZ == std::fwrite( pixels.data( ), pixels.size( ), 1UZ, video_pipe_ ) );
    ++video_frame_count_;

    return utils::success( );
}

auto FlewIdApp::read_capture( ) -> utils::Result< std::vector< uint8 > >
{
    LTB_CHECK_VALID( capture_buffer_.is_initialized( ) );

    VK_CHECK( gpu_.device( ).get( ).waitIdle( ) );

    auto const extent      = presentation_.swapchain( ).settings( ).extent;
    auto const pixel_count = static_cast< size_t >( extent.width ) * extent.height;

    LTB_CHECK( auto const* const data, capture_buffer_.memory( ).map( ) );
    auto pixels = std::vector< uint8 >( data, data + ( pixel_count * 4UZ ) );
    capture_buffer_.memory( ).unmap( );

    return pixels;
}

auto FlewIdApp::write_screenshot( ) -> utils::Result< void >
{
    auto const extent      = presentation_.swapchain( ).settings( ).extent;
    auto const pixel_count = static_cast< size_t >( extent.width ) * extent.height;

    LTB_CHECK( auto pixels, this->read_capture( ) );

    // The swapchain stores blue first.
    auto const format = presentation_.swapchain( ).image_format( );
    if ( vk::Format::eB8G8R8A8Unorm == format || vk::Format::eB8G8R8A8Srgb == format )
    {
        for ( auto pixel = 0UZ; pixel < pixel_count; ++pixel )
        {
            std::swap( pixels[ ( pixel * 4UZ ) + 0UZ ], pixels[ ( pixel * 4UZ ) + 2UZ ] );
        }
    }
    for ( auto pixel = 0UZ; pixel < pixel_count; ++pixel )
    {
        pixels[ ( pixel * 4UZ ) + 3UZ ] = 255U;
    }

    auto const& file = app_options( ).screenshot;
    LTB_CHECK( write_png( file, { extent.width, extent.height }, pixels ) );
    spdlog::info( "Wrote {}", file.string( ) );

    return utils::success( );
}

auto FlewIdApp::render( ) -> utils::Result< void >
{
    LTB_CHECK(
        auto const maybe_frame,
        graphics_cmd_and_sync_.start_frame( presentation_.swapchain( ) )
    );

    if ( maybe_frame.has_value( ) )
    {
        auto const frame = maybe_frame.value( );

        LTB_CHECK( this->update_camera_uniforms( frame ) );
        LTB_CHECK( this->record_render_commands( frame ) );

        auto wait_until_signaled = std::vector< vlk::objs::SemaphoreAndStage >{ };

        if ( compute_semaphore_ )
        {
            wait_until_signaled.push_back( {
                .semaphore = compute_semaphore_.value( ),
                .stage     = vk::PipelineStageFlagBits::eVertexInput,
            } );
            compute_semaphore_ = std::nullopt;
        }
        wait_until_signaled.push_back( {
            .semaphore = frame.image_semaphore,
            .stage     = vk::PipelineStageFlagBits::eColorAttachmentOutput,
        } );

        LTB_CHECK(
            auto const& render_finished_semaphore,
            graphics_cmd_and_sync_.get_present_semaphore( frame )
        );

        LTB_CHECK( graphics_cmd_and_sync_.end_frame(
            frame,
            std::move( wait_until_signaled ),
            { render_finished_semaphore },
            graphics_and_compute_queue_
        ) );

        LTB_CHECK( graphics_cmd_and_sync_
                       .present_frame( frame, presentation_.swapchain( ), present_queue_ ) );

        graphics_cmd_and_sync_.increment_frame( );

        if ( this->is_recording( ) )
        {
            LTB_CHECK( this->write_video_frame( ) );
        }
    }

    return utils::success( );
}

auto FlewIdApp::update_camera_uniforms( vlk::objs::FrameInfo const& frame )
    -> utils::Result< void >
{
    if ( camera_frames_updated_.contains( frame.frame_index ) )
    {
        return utils::success( );
    }

    auto const camera_ubo = DisplayUniforms{
        .clip_from_world = camera_.simple_render_params( ).clip_from_world,
        .viewport_size   = glm::vec2( glfw_window_.framebuffer_size( ) ),
        .diameter        = particles_->radius( ) * 2.0F,
        .max_speed       = max_display_speed_,
        .tunnel_size     = particles_->tunnel( ).size,
        .solid_size      = particles_->settings( ).solid_field_size,
        .field_size      = particles_->field_size( ),

        .field_mode        = field_mode_,
        .field_range       = field_range_,
        .reference_density = particles_->settings( ).fill.packing_fraction,
        // The mean squared thermal speed.
        .reference_temperature = thermal_speed_ * thermal_speed_,
        .particle_mode         = particle_mode_,
        .particle_brightness   = particle_brightness_,
        .buffer_width
        = ( step_params_.buffers && step_params_.flow_speed > 0.0F ) ? step_params_.buffer_width : 0.0F,
    };

    auto display_ubo = camera_ubo;
    if ( FluidModel::Sph == model_ )
    {
        // Show the pressure relative to the dynamic pressure of the incoming flow.
        auto const speed = std::max( step_params_.flow_speed, 0.25F );

        display_ubo.dynamic_pressure = 0.5F * speed * speed;
    }

    LTB_CHECK_VALID( frame.frame_index < camera_ubo_.layout( ).ranges.size( ) );
    auto const& memory_range = camera_ubo_.layout( ).ranges[ frame.frame_index ];

    auto* const dst_data = camera_ubo_.mapped_data( ) + memory_range.offset;
    LTB_CHECK_VALID( memory_range.size >= sizeof( display_ubo ) );
    LTB_CHECK_VALID( std::memcpy( dst_data, &display_ubo, sizeof( display_ubo ) ) == dst_data );

    camera_frames_updated_.emplace( frame.frame_index );

    return utils::success( );
}

auto FlewIdApp::record_render_commands( vlk::objs::FrameInfo const& frame )
    -> utils::Result< void >
{
    VK_CHECK( frame.command_buffer.begin( vk::CommandBufferBeginInfo{ } ) );

    LTB_CHECK( presentation_.begin_render_pass( {
        .command_buffer    = frame.command_buffer,
        .image_index       = frame.image_index,
        .color_clear_value = { 0.02F, 0.02F, 0.03F, 1.0F },
    } ) );

    // The tunnel and solids.
    background_.bind( frame.command_buffer );
    LTB_CHECK( background_.bind_descriptor_sets( frame ) );
    frame.command_buffer.draw( 4U, 1U, 0U, 0U );

    if ( !show_particles_ )
    {
        imgui_.render( frame.command_buffer );
        frame.command_buffer.endRenderPass( );
        return this->record_capture_commands( frame );
    }

    // The particles.
    graphics_.bind( frame.command_buffer );

    LTB_CHECK( graphics_.bind_descriptor_sets( frame ) );

    constexpr auto first_binding  = 0U;
    auto const     vertex_buffers = std::array{ particles_->particle_buffer( ) };
    auto const     vertex_offsets = std::array< vk::DeviceSize, 1 >{ 0U };
    frame.command_buffer.bindVertexBuffers( first_binding, vertex_buffers, vertex_offsets );

    constexpr auto instance_count = 1U;
    constexpr auto first_vertex   = 0U;
    constexpr auto first_instance = 0U;
    frame.command_buffer
        .draw( particles_->particle_count( ), instance_count, first_vertex, first_instance );

    imgui_.render( frame.command_buffer );

    frame.command_buffer.endRenderPass( );

    return this->record_capture_commands( frame );
}

auto FlewIdApp::record_capture_commands( vlk::objs::FrameInfo const& frame )
    -> utils::Result< void >
{
    if ( this->is_recording( ) || ( finishing_ && !app_options( ).screenshot.empty( ) ) )
    {
        // Copy the rendered image into a buffer the CPU can read.
        auto const extent = presentation_.swapchain( ).settings( ).extent;
        auto const size   = static_cast< vk::DeviceSize >( extent.width ) * extent.height * 4U;

        if ( !capture_buffer_.is_initialized( ) || capture_buffer_.layout( ).total_size != size )
        {
            VK_CHECK( gpu_.device( ).get( ).waitIdle( ) );
            capture_buffer_.reset( );
            LTB_CHECK( capture_buffer_.initialize( {
                .layout       = { .total_size = size },
                .buffer_usage = vk::BufferUsageFlagBits::eTransferDst,
                .memory_properties = vk::MemoryPropertyFlagBits::eHostVisible
                                   | vk::MemoryPropertyFlagBits::eHostCoherent,
            } ) );
        }

        LTB_CHECK_VALID( frame.image_index < presentation_.swapchain( ).images( ).size( ) );
        auto const image = presentation_.swapchain( ).images( )[ frame.image_index ];

        auto const subresource_range = vk::ImageSubresourceRange{ }
                                           .setAspectMask( vk::ImageAspectFlagBits::eColor )
                                           .setLevelCount( 1U )
                                           .setLayerCount( 1U );

        auto const to_transfer = vk::ImageMemoryBarrier{ }
                                     .setSrcAccessMask( vk::AccessFlagBits::eColorAttachmentWrite )
                                     .setDstAccessMask( vk::AccessFlagBits::eTransferRead )
                                     .setOldLayout( vk::ImageLayout::ePresentSrcKHR )
                                     .setNewLayout( vk::ImageLayout::eTransferSrcOptimal )
                                     .setSrcQueueFamilyIndex( vk::QueueFamilyIgnored )
                                     .setDstQueueFamilyIndex( vk::QueueFamilyIgnored )
                                     .setImage( image )
                                     .setSubresourceRange( subresource_range );
        frame.command_buffer.pipelineBarrier(
            vk::PipelineStageFlagBits::eColorAttachmentOutput,
            vk::PipelineStageFlagBits::eTransfer,
            { },
            { },
            { },
            to_transfer
        );

        auto const region = vk::BufferImageCopy{ }
                                .setImageSubresource(
                                    vk::ImageSubresourceLayers{ }
                                        .setAspectMask( vk::ImageAspectFlagBits::eColor )
                                        .setLayerCount( 1U )
                                )
                                .setImageExtent( { extent.width, extent.height, 1U } );
        frame.command_buffer.copyImageToBuffer(
            image,
            vk::ImageLayout::eTransferSrcOptimal,
            capture_buffer_.buffer( ).get( ),
            region
        );

        auto const to_present = vk::ImageMemoryBarrier{ }
                                    .setSrcAccessMask( vk::AccessFlagBits::eTransferRead )
                                    .setDstAccessMask( { } )
                                    .setOldLayout( vk::ImageLayout::eTransferSrcOptimal )
                                    .setNewLayout( vk::ImageLayout::ePresentSrcKHR )
                                    .setSrcQueueFamilyIndex( vk::QueueFamilyIgnored )
                                    .setDstQueueFamilyIndex( vk::QueueFamilyIgnored )
                                    .setImage( image )
                                    .setSubresourceRange( subresource_range );
        frame.command_buffer.pipelineBarrier(
            vk::PipelineStageFlagBits::eTransfer,
            vk::PipelineStageFlagBits::eBottomOfPipe,
            { },
            { },
            { },
            to_present
        );
    }

    VK_CHECK( frame.command_buffer.end( ) );

    return utils::success( );
}

} // namespace ltb::flew
