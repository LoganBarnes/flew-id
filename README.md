# Flew Id

A 2D GPU particle wind tunnel for building an intuition of the fluid dynamics of flight.
See [VISION.md](VISION.md) for the goals of the project.

Hundreds of thousands to millions of particles collide with each other and with solid
objects (a wing, a cylinder, ...) entirely in Vulkan compute shaders.

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

# Run a fixed number of steps, verify the particles, and save the last frame.
./build/src/ltb/apps/Release/flew-id --scenario Cylinder --steps 50000 --check --screenshot out.png

# The same verification with no window.
./build/src/ltb/apps/Release/flew-check --scenario Venturi --steps 10000 --particles 1000000

# Record a video (requires ffmpeg): 400 simulation steps per frame, background only.
./build/src/ltb/apps/Release/flew-id --scenario Cylinder --field Vorticity --hide-particles \
    --hide-gui --steps 120000 --steps-per-frame 400 --video cylinder.mp4
```

Scenarios: `Box`, `Cylinder`, `Airfoil`, `Plate`, `Venturi`, `LongVenturi`. Use `--help` for
all options.

The venturi scenarios choke at the default flow speed and throat (pressure builds up before
the constriction and the flow leaves it as a jet). For the textbook picture of faster flow
and lower pressure in the throat, use a slower flow and a wider throat, e.g.
`--scenario LongVenturi --flow 0.2 --throat 0.5`.

The background can be colored by `Speed`, `Pressure`, `Density`, `Vorticity`, or
`Temperature` (`--field`, or the Display section of the settings window). These are
running averages over many particles and steps: individual particles move randomly, and
it is the averages that behave like a fluid.

## How it works

Each simulation step is a chain of compute passes (`res/shaders/`, driven by
`src/ltb/flew/particle_system.cpp`):

1. Count the particles in each cell of a uniform grid (cells are one particle diameter wide).
2. Prefix sum the counts to find where each cell's particles start.
3. Scatter the particles into a second buffer sorted by cell.
4. For every particle, apply spring/damping forces from overlapping particles in the
   neighboring cells (`interaction.glsl`), integrate, and collide with the tunnel walls and
   the signed distance field of the solids (`solid.glsl`).

When there is flow, the left and right ends of the tunnel are connected and particles
passing through a thin region on the left are reset to the incoming flow conditions.
With `--respawn` (or "Respawn at tunnel ends" in the settings window) every particle that
crosses an end of the tunnel is also replaced by a new particle of the undisturbed flow, as
if the tunnel continued far beyond both ends.
