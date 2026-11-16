# Flew Id

A 2D GPU particle wind tunnel (C++23, Vulkan compute) for building intuition about the fluid
dynamics of flight. Goals are in `VISION.md`; usage and the step pipeline are in `README.md`.

## Build

Needs a C++23 compiler, CMake 3.23+, Ninja, the Vulkan SDK (`VULKAN_SDK` set, `glslc`
available), TBB, X11 dev libraries, and ffmpeg for `--video`. Other dependencies come from CPM.

```bash
git submodule update --init
cmake -S . -B cmake-build-default -G "Ninja Multi-Config" \
    -DLTB_FLEW_USE_STRICT_FLAGS=ON -DLTB_VLK_USE_STRICT_FLAGS=ON -DLTB_FLEW_BUILD_TESTS=ON \
    -DCPM_SOURCE_CACHE=$HOME/.cache/CPM
cmake --build cmake-build-default --config Debug --target flew-id flew-check flew-tests
```

- The owner's CLion profile builds in `cmake-build-default/` with these same options. Use
  that directory (`-B cmake-build-default`) instead of creating a second one.
- `CPM_SOURCE_CACHE` must be an absolute path when set in an IDE: CLion does not expand
  `$HOME`, and CMake then creates a literal `$HOME/` directory.
- ltb-vlk is built from the local `ltb-vlk/` submodule automatically (`cmake/ThirdParty.cmake`
  sets `CPM_LtbVlk_SOURCE` when the checkout exists). No flag is needed.

- `cmake` may not be on `PATH`. CLion's bundled one works
  (`~/.local/share/JetBrains/Toolbox/apps/clion/bin/cmake/linux/x64/bin`).
- If `VULKAN_SDK` is unset, source the SDK's `setup-env.sh` first.
- Strict flags are warnings-as-errors; keep them on.
- Shaders (`res/shaders/*.{comp,vert,frag}`) compile to `<build dir>/spirv/` as part of the build.
  `*.glsl` files are includes.

## Verify changes

Unit tests cover the CPU-side code (`tests/`: solids and distance fields, particle fill and
stats, scenarios). The GPU simulation is verified by running it:

```bash
# Unit tests (GoogleTest, needs -DLTB_FLEW_BUILD_TESTS=ON).
ctest --test-dir cmake-build-default -C Debug

# Headless health check (no window). Run for every scenario and both models
# (--model Gas, --model SPH) after simulation changes.
./cmake-build-default/src/ltb/apps/Debug/flew-check --model SPH --scenario Airfoil --steps 10000

# Per-pass GPU timings (timestamp queries), averaged over the run. Use this before and
# after performance changes.
./cmake-build-default/src/ltb/apps/Debug/flew-check --model SPH --scenario Airfoil --steps 10000 --profile

# Look at the result: run fast (--check uses 50 steps per update), save the last frame,
# then open the PNG.
./cmake-build-default/src/ltb/apps/Debug/flew-id --scenario Airfoil --field Pressure --hide-particles \
    --steps 150000 --check --screenshot /tmp/out.png
```

- Scenarios: `Box`, `Cylinder`, `Airfoil`, `Plate`, `Venturi`, `LongVenturi` (an 8x2 tunnel;
  the others are 4x2, see `scenario_tunnel`).
  Fields: `None`, `Speed`, `Pressure`, `Density`, `Vorticity`, `Temperature`.
- Without `--check` or `--steps-per-frame`, `flew-id` runs at interactive pace (4 steps per
  update), so large `--steps` values take minutes. That is not a hang.
- Flow takes roughly 70k steps to cross the tunnel at the defaults, so features such as wakes
  need 100k+ steps to develop.
- Debug builds enable Vulkan validation. Any `Validation Error` in the output is a failure.
  The `libvulkan_gfxstream.so ... treated as a 1.0 ICD` debug line is harmless.

## Layout

- `src/ltb/flew/` library:
  - `particle_system.*` owns the GPU buffers and records the compute chain for one step,
    for either fluid model (`FluidModel::Gas` or `FluidModel::Sph`).
  - `particle.*` initial fill and CPU-side stats/checks.
  - `solid_scene.*` polygons/circles, signed distance field, NACA airfoil.
  - `scenarios.*` scenario definitions.
- `src/ltb/apps/flew_id/` interactive app (GUI, rendering, screenshot, video).
- `src/ltb/apps/flew_check/` headless check.
- `res/shaders/` compute passes and display shaders.
- `tests/` CPU unit tests, one file per library source file.
- `ltb-vlk/` git submodule: the Vulkan/GLFW/ImGui library this is built on.

## Things that are easy to get wrong

- Structs shared with GLSL must match byte for byte: `StepConstants` (`particle_system.cpp`)
  with the push constant block in `particle.glsl` (128 byte limit), and `DisplayUniforms`
  (`app.cpp`) with `display.glsl`.
- `StepConstants` is exactly 128 bytes. Adding a field means removing or packing another.
- Shader constants mirrored on the CPU: `SCAN_BLOCK_SIZE` (`particle.glsl`) and
  `SCAN_THREADS` (`grid_scan_up.comp`) with `scan_block_size`/`scan_threads`, and
  `SPH_CELL_REACH` (`sph.glsl`) with `sph_cell_subdivision` (all in `particle_system.cpp`).
- SPH grid cells are half the kernel's support (searched 5x5), so the field scale is doubled
  internally to keep field cells the same size. Shaders target Vulkan 1.1 (the prefix sum
  uses subgroup arithmetic).
- Every compute pipeline shares the same six storage buffer bindings (`particle.glsl`).
  Adding a binding means updating `initialize_pipeline` in `particle_system.cpp`.
- The default time step and contact stiffness are tied together (`default_step_params`).
  Softer or larger steps let particles pass through each other.
- Scenario defaults come from `default_scenario_settings`. The venturi scenarios use flow 0.2
  and throat 0.5 because they choke at the general defaults (flow 0.5, throat 1/3).
- Buffer zones (`StepParams::buffers`, on by default) reset velocities at both ends of the
  tunnel. Moving particles sideways in the buffers to even out density was tried (a drift
  down the gradient of the time-averaged density) and removed: the averaged density lags, the
  correction overshoots, and the simulation blows up.
- The gas model is compressible and noisy. Only the averaged fields behave like a fluid;
  keep the flow speed below the thermal speed. Slowing the flow to make it less
  compressible buries the pressure signal in noise (it scales with speed squared).
- SPH: the particle mass is normalized so evenly spaced particles sum to a density of
  exactly 1 (`ParticleSystem::initialize`). A 1% density error is a large pressure error
  because the fluid is stiff, so pressure is displayed relative to the measured inflow
  pressure (`undisturbed_pressure` in `field.frag`), not a nominal value.
- SPH fields are averaged over the time sound takes to cross the tunnel height
  (`default_sph_step_params`). Shorter averages mostly show sound waves bouncing between the
  walls, not the flow. Starting the flow gradually did not reduce those waves.
- Caching neighbors in shared memory was tried for the SPH passes and was slower on both
  NVIDIA and AMD: the sorted neighbor reads already hit the L1 cache.
- SPH wall handling (`sph.glsl`): walls contribute density and a pressure force using
  tabulated integrals of the kernel beyond a flat wall. The tables are specific to the
  Wendland C2 kernel; changing the kernel means recomputing them.

## ltb-vlk

`ltb-vlk` is the owner's own library and is meant to be edited here: fix bugs and improve it
directly in the submodule. CMake builds against the local `ltb-vlk/` checkout automatically.

- Commit submodule changes on a branch (e.g. `flew-id`) and push that branch before pushing
  a flew-id commit that moves the submodule pointer.
- Don't push to ltb-vlk `main` without being asked; the owner decides when branches merge. Their `cfd` repo also uses ltb-vlk, so call
  out API-breaking changes.

## Conventions

Follow the existing style (`.clang-format`, `.clang-tidy`): trailing return types,
`utils::Result` with `LTB_CHECK` / `LTB_CHECK_VALID` / `VK_CHECK`, `*Settings` aggregates with
designated initializers, fixed-width aliases (`float32`, `uint32`), and the file header banner.

## Known gaps

- No scrubbable playback format yet (video only), no Coanda scenario, no 3D, no web port.
- The closed `Box` scenario slowly gains energy (about 7% over 80k steps); there is no
  thermostat outside of the inflow region.
- SPH has no particle shifting or density diffusion, so there is some acoustic noise in the
  pressure field, and flow separates readily in the venturis' widening sections.
- Density is not relaxed at the tunnel ends (only velocity), so uneven density wraps around.
  A real fix needs open ends that add and remove particles.
- Field cells partly covered by a solid read as low density/pressure (a thin false rim).
