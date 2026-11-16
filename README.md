# Flew Id

A 2D GPU particle wind tunnel for building an intuition of the fluid dynamics of flight.
See [VISION.md](VISION.md) for the goals of the project.

Hundreds of thousands to millions of particles collide with each other and with solid
objects (a wing, a cylinder, ...) entirely in Vulkan compute shaders.

## Fluid models

There are two ways of simulating the fluid (`--model`, or "Fluid model" in the settings
window):

- **Gas** (the default): the particles are disks with random thermal motion that bounce
  off each other, like the molecules of a gas. Pressure, viscosity, and the speed of
  sound all emerge from the collisions. Good for seeing where fluid behavior comes from,
  but noisy: flow properties only appear when averaged over many particles, and the gas
  is noticeably compressible at flow speeds fast enough to see clearly.
- **SPH** (smoothed particle hydrodynamics): each particle is a parcel of fluid with no
  random motion. Pressure is computed from how closely the parcels are packed, and the
  viscosity and speed of sound are settings. The flow is smooth and nearly incompressible,
  so this is the model for clean pictures of lift, wakes, and the venturi effect. In this
  model "Pressure" shows the pressure coefficient: 1 (red) where the flow is brought to
  rest, negative (blue) where it is faster than the incoming flow.

## Building

Requirements: a C++23 compiler, CMake 3.23+, Ninja, the Vulkan SDK (with `glslc`), TBB, and
X11 development libraries. Everything else is downloaded by CMake.

```bash
git clone --recurse-submodules https://github.com/LoganBarnes/flew-id.git
cd flew-id

cmake -S . -B build -G "Ninja Multi-Config" -DLTB_FLEW_USE_STRICT_FLAGS=ON
cmake --build build --config Release
```

Add `-DLTB_FLEW_BUILD_TESTS=ON` to build the unit tests (`flew-tests`), then run them with
`ctest --test-dir build -C Release`.

[ltb-vlk](https://github.com/LoganBarnes/ltb-vlk) is included as a git submodule and built
from that local copy, so changes made in `ltb-vlk/` are picked up immediately.

## Running

```bash
# The interactive app.
./build/src/ltb/apps/Release/flew-id --scenario Airfoil --angle 8

# The same scenario with smooth parcels of fluid, colored by pressure.
./build/src/ltb/apps/Release/flew-id --model SPH --scenario Airfoil --field Pressure

# Run a fixed number of steps, verify the particles, and save the last frame.
./build/src/ltb/apps/Release/flew-id --scenario Cylinder --steps 50000 --check --screenshot out.png

# The same verification with no window.
./build/src/ltb/apps/Release/flew-check --scenario Venturi --steps 10000 --particles 1000000

# Print the GPU time spent in each compute pass.
./build/src/ltb/apps/Release/flew-check --model SPH --scenario Airfoil --steps 10000 --profile

# Record a video (requires ffmpeg): 400 simulation steps per frame, background only.
./build/src/ltb/apps/Release/flew-id --scenario Cylinder --field Vorticity --hide-particles \
    --hide-gui --steps 120000 --steps-per-frame 400 --video cylinder.mp4
```

Scenarios: `Box`, `Cylinder`, `Airfoil`, `Plate`, `Venturi`, `LongVenturi`. Use `--help` for
all options.

The venturi scenarios start with a slower flow (0.2) and a wider throat (half the tunnel's
height) than the other scenarios' defaults. The fluid is compressible, so a fast flow
through a narrow throat chokes: pressure builds up before the constriction and the flow
leaves it as a jet (try `--flow 0.5 --throat 0.33`).

The background can be colored by `Speed`, `Pressure`, `Density`, `Vorticity`, or
`Temperature` (`--field`, or the Display section of the settings window). These are
running averages over many particles and steps: individual particles move randomly, and
it is the averages that behave like a fluid.

## How it works

Each simulation step is a chain of compute passes (`res/shaders/`, driven by
`src/ltb/flew/particle_system.cpp`):

1. Count the particles in each cell of a uniform grid (gas: cells are one particle diameter
   wide; SPH: half the kernel's support, searching two cells in each direction).
2. Prefix sum the counts to find where each cell's particles start.
3. Scatter the particles into a second buffer sorted by cell.
4. Gas: for every particle, apply spring/damping forces from overlapping particles in the
   neighboring cells (`interaction.glsl`) and integrate (`collide_integrate.comp`).
   SPH: compute the density at every particle from its neighbors (`sph_density.comp`), then
   apply pressure and viscous forces and integrate (`sph_integrate.comp`). Walls count as
   fluid at rest, which keeps the pressure correct next to them and makes the fluid stick.
5. Apply the tunnel ends, walls, and the signed distance field of the solids
   (`boundary.glsl`, `solid.glsl`).

When there is flow, the left and right ends of the tunnel are connected. Buffer zones at
both ends (lightly tinted in the app) randomly reset particles to the undisturbed flow, more
often the closer they are to the end, and every particle crossing an end becomes a new
particle of that flow. The outlet buffer absorbs wakes before they can wrap around and the
inlet buffer delivers a uniform velocity, like the diffuser and settling chamber of a real
wind tunnel. The buffers reset velocities only: particles keep their positions, so uneven
density is still carried around the ends.

`--no-buffers` restores the original behavior (resets only in a thin region at the inlet),
optionally with `--respawn` to also replace particles as they cross the ends.
