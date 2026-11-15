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
```

Scenarios: `Box`, `Cylinder`, `Airfoil`, `Plate`, `Venturi`. Use `--help` for all options.

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
