# TODO

Open work, roughly in priority order within each section. Measure performance changes with
`flew-check --profile` (Release for timing, Debug for validation) and verify every scenario
with both models. Things already tried and rejected are listed at the end so they aren't
retried blindly.

Current cost at 200k particles (RTX 4050, GPU time per step): Gas about 0.09 ms, SPH about
0.29 ms, mostly `sph_integrate` (about 0.15) and `sph_density` (about 0.10).

## Physics and correctness

- [ ] **SPH particle shifting or density diffusion (delta-SPH).** Reduces the acoustic
      noise in the pressure field and the early flow separation in the venturis' widening
      sections (the pressure stays low after the throat).
- [ ] **Open tunnel ends.** Density is not relaxed at the ends (only velocity), so uneven
      density wraps around. The real fix is inflow and outflow that add and remove particles.
- [ ] **Box energy drift.** The closed `Box` gains about 7% energy over 80k steps. Add a
      weak thermostat in the bulk.
- [ ] **False rim at solids.** Field cells partly covered by a solid read as low density and
      pressure. Normalize by the free area fraction from the distance field.
- [ ] **Stricter `flew-check`.** It only fails on invalid, out of bounds, and overlap
      (`max_overlap`). Add thresholds on energy drift and max speed to catch blow-ups. Gas
      `max_overlap` is about 0.19, so the stiffness and time step are near their limit.

## Display

- [ ] **Pressure reference.** The tunnel-wide pressure drop (from drag and the buffers) tints
      everything downstream blue, which competes with the pressure around the solid.

## Performance

- [ ] **Sort less often.** Count, scan, and scatter run every step (about 0.035 ms of SPH's
      0.29). Re-sorting every few steps with a skin distance (Verlet-style) could save most
      of that, but the SPH passes dominate.
- [ ] **Larger SPH time step.** The step is set by the speed of sound (10x the fastest flow).
      A lower ratio is faster and less noisy but more compressible. This is a physics
      tradeoff, so measure the density error before changing it.

## Features (see `VISION.md`)

- [ ] **Lift and drag.** Integrate pressure and viscous forces over the solids, and plot
      lift and drag coefficients against angle of attack. This is the biggest gain for
      building intuition about flight.
- [ ] **Streamlines, dye, or tracer particles.** Particles are hidden by default in SPH
      (packed tightly they only draw a moiré pattern) and are re-sorted every step, so tracers
      need their own buffer advected through the velocity field.
- [ ] **Interactive solids.** Drag the airfoil and change its angle live.
- [ ] **Coanda scenario.**
- [ ] **Scrubbable playback.** Record the coarse field grid (small) instead of video, so it
      can be replayed and scrubbed without re-simulating.
- [ ] **3D and a web (WebGPU) port.**

## Engineering

- [ ] **CI.** GitHub Actions for the build and unit tests, plus a short `flew-check` smoke
      run on lavapipe (software Vulkan).
- [ ] **Split `app.cpp`** (about 1.2k lines) into GUI, rendering, and capture.
- [ ] **CPU reference for the grid passes.** Compare the GPU count, scan, and scatter
      against a CPU version in a test.
- [ ] **Parameter sweeps.** A script that sweeps scenario and step parameters (flow speed,
      throat, sound ratio) and reports `flew-check` stats, instead of tuning by hand.

## Tried and rejected

- **Shared-memory neighbor cache for the SPH passes.** Slower on NVIDIA (integrate 0.15 to
  0.26 ms) and AMD (0.25 to 0.40 ms): the sorted neighbor reads already hit the L1 cache.
- **fp16 positions.** At a tunnel width of 4 the precision is about 0.002, close to the
  particle radius (about 0.003).
- **Starting the SPH flow gradually.** Made no measurable difference to the sound waves.
- **Drifting particles down the time-averaged density gradient in the buffers.** The average
  lags, the correction overshoots, and the simulation blows up.
