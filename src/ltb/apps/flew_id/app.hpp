// ///////////////////////////////////////////////////////////////////////////////////////
// A Logan Thomas Barnes project
// ///////////////////////////////////////////////////////////////////////////////////////
#pragma once

// project
#include "ltb/cam/camera_2d.hpp"
#include "ltb/exec/update_loop.hpp"
#include "ltb/flew/particle_system.hpp"
#include "ltb/flew/scenarios.hpp"
#include "ltb/gui/imgui_glfw_vulkan_setup.hpp"
#include "ltb/vlk/objs/vulkan_buffer.hpp"
#include "ltb/vlk/objs/vulkan_command_and_sync.hpp"
#include "ltb/vlk/objs/vulkan_gpu.hpp"
#include "ltb/vlk/objs/vulkan_graphics_pipeline.hpp"

// standard
#include <unordered_set>

namespace ltb::flew
{

/// \brief Options set from the command line before the app is created.
struct AppOptions
{
    /// \brief Overrides the default number of particles when not zero.
    uint32 particle_count = 0U;
    /// \brief The scenario to start with and its settings.
    Scenario scenario = Scenario::Airfoil;
    /// \brief Overrides of the scenario's default settings.
    std::optional< float32 > angle_degrees   = std::nullopt;
    std::optional< float32 > flow_speed      = std::nullopt;
    std::optional< float32 > throat_fraction = std::nullopt;
    /// \brief Only reset particles in a thin region at the inlet instead of
    ///        in buffer zones at both ends of the tunnel.
    bool no_buffers = false;
    /// \brief The background coloring to start with (None, Speed, Pressure, ...).
    std::string field = "Speed";
    /// \brief Start with the particles hidden.
    bool hide_particles = false;
    /// \brief Replace particles leaving the ends of the tunnel with new incoming particles.
    bool respawn = false;
    /// \brief Exit after this many simulation steps when not zero.
    uint64 max_steps = 0UZ;
    /// \brief Read the particles back after `max_steps` and verify the simulation is healthy.
    bool check = false;
    /// \brief Write the last rendered frame to this PNG file before exiting after `max_steps`.
    std::filesystem::path screenshot = { };
    /// \brief Record every rendered frame to this video file (requires ffmpeg).
    std::filesystem::path video     = { };
    uint32                video_fps = 60U;
    /// \brief The number of simulation steps per update (and per video frame) when not zero.
    uint32 steps_per_update = 0U;
    /// \brief Don't draw the settings window.
    bool hide_gui = false;
    /// \brief Set by the app if the check (or the simulation itself) failed.
    bool failed = false;
};

auto app_options( ) -> AppOptions&;

class FlewIdApp
{
public:
    explicit FlewIdApp( window::GlfwContext& glfw_context, window::GlfwWindow& glfw_window );

    auto initialize( ) -> utils::Result< exec::UpdateLoopStatus >;

    [[nodiscard( "Const getter" )]]
    auto is_initialized( ) const -> bool;

    auto fixed_step_update( exec::UpdateLoopStatus const& status ) -> exec::UpdateRequests;
    auto frame_update( exec::UpdateLoopStatus const& status ) -> exec::UpdateRequests;

    auto configure_gui( ) -> void;

    auto on_resize( glm::ivec2 size ) -> utils::Result< void >;
    auto clean_up( ) -> utils::Result< void >;

private:
    window::GlfwContext& glfw_context_;
    window::GlfwWindow&  glfw_window_;

    vlk::objs::VulkanGpu gpu_                        = { glfw_context_, glfw_window_ };
    vk::Queue            graphics_and_compute_queue_ = nullptr;
    vk::Queue            present_queue_              = nullptr;

    vlk::objs::VulkanPresentation presentation_ = { gpu_ };
    gui::ImguiGlfwVulkanSetup     imgui_        = { glfw_window_, gpu_, presentation_ };

    vlk::objs::VulkanCommandAndSync compute_cmd_and_sync_ = { gpu_ };
    std::unique_ptr< ParticleSystem > particles_          = nullptr;
    StepParams                      step_params_          = { };
    int32                           substeps_             = 4;
    uint64                          step_count_           = 0UZ;
    bool                            paused_               = false;
    bool                            single_step_          = false;

    ParticleStats initial_stats_ = { };
    bool          check_failed_  = false;

    /// \brief True once `max_steps` has been reached. The next frame is the last.
    bool finishing_ = false;
    /// \brief Receives a copy of the rendered image when a screenshot is requested.
    vlk::objs::VulkanBuffer capture_buffer_ = { gpu_ };

    /// \brief The standard input of the ffmpeg process encoding the video (if recording).
    std::FILE*   video_pipe_        = nullptr;
    vk::Extent2D video_extent_      = { };
    uint64       video_frame_count_ = 0UZ;

    std::optional< vk::Semaphore > compute_semaphore_ = std::nullopt;

    Scenario         scenario_          = Scenario::Airfoil;
    ScenarioSettings scenario_settings_ = { };
    uint32           particle_count_    = ParticleFillSettings{ }.particle_count;
    float32          thermal_speed_     = ParticleFillSettings{ }.thermal_speed;
    /// \brief Set by the GUI to restart the simulation with the current scenario settings.
    bool restart_requested_ = false;

    /// \brief Draws the tunnel background and solids.
    vlk::objs::VulkanGraphicsPipeline background_            = { gpu_, presentation_ };
    vlk::objs::VulkanGraphicsPipeline graphics_              = { gpu_, presentation_ };
    vlk::objs::VulkanCommandAndSync   graphics_cmd_and_sync_ = { gpu_ };

    vlk::objs::VulkanBuffer      camera_ubo_            = { gpu_ };
    cam::Camera2d                camera_                = { };
    std::unordered_set< uint32 > camera_frames_updated_ = { };

    float32 max_display_speed_   = 1.0F;
    int32   field_mode_          = 1;
    float32 field_range_         = 0.5F;
    bool    show_particles_      = true;
    int32   particle_mode_       = 1;
    float32 particle_brightness_ = 0.35F;

    bool initialized_ = false;

    auto initialize_gpu_presentation( ) -> utils::Result< FlewIdApp* >;
    auto initialize_simulation( ) -> utils::Result< FlewIdApp* >;
    /// \brief (Re)creates the particles and solids for the current scenario.
    auto start_scenario( ) -> utils::Result< void >;
    /// \brief Points the background pipeline at the current solids.
    auto update_solid_descriptors( ) -> utils::Result< void >;
    auto initialize_display_pipeline( ) -> utils::Result< FlewIdApp* >;
    auto initialize_camera( ) -> utils::Result< FlewIdApp* >;

    auto compute( ) -> utils::Result< void >;
    auto check_particles( ) -> utils::Result< void >;
    auto finish( ) -> utils::Result< void >;
    auto write_screenshot( ) -> utils::Result< void >;
    /// \brief Copies the most recently captured image (BGRA or RGBA, top row first).
    auto read_capture( ) -> utils::Result< std::vector< uint8 > >;

    /// \brief Runs the simulation forward by the current number of steps per update.
    auto advance( exec::UpdateRequests requests ) -> exec::UpdateRequests;

    [[nodiscard( "Const getter" )]]
    auto is_recording( ) const -> bool;
    auto start_recording( ) -> utils::Result< void >;
    auto stop_recording( ) -> void;
    auto write_video_frame( ) -> utils::Result< void >;

    auto render( ) -> utils::Result< void >;
    auto update_camera_uniforms( vlk::objs::FrameInfo const& frame ) -> utils::Result< void >;
    auto record_render_commands( vlk::objs::FrameInfo const& frame ) -> utils::Result< void >;
    /// \brief Ends the frame's commands, copying the rendered image first if it is needed.
    auto record_capture_commands( vlk::objs::FrameInfo const& frame ) -> utils::Result< void >;
};

} // namespace ltb::flew
