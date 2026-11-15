# Flew Id

Flew (flight) - Id (Instinctual) -> "Fluid"

This project should create an instinctual understanding of fluid dynamics as they pertain to flight.

## Concept

A particle fluid simulation used to demonstrate aerodynamic flight principles. This should make the understanding of common fluid concepts more intuitive (Bernoulli, viscosity, turbulence, Coanda effect, etc.). The particle system should be run on the GPU with as many particles as reasonable for the system. The particles don't need to do much other than impact each other and any other solid objects added to the scene (walls, wings, etc.).

An existing project has been symlinked in this directory that could serve as a reference or starting point for this project. https://gitlab.com/LoganBarnes/flight is also contains an example of a particle simulation in the AirView. It doesn't have to be in C/C++, but it should run pretty fast. It would be nice to extend this to the web but it probably won't be as performant for most people on normal computers. Maybe some of the lighter-weight examples could be run on the web with fewer-larger particles so people can still run on laptops and phones. The primary purpose however, should be something that allows us to get the most performance possible.

It would also be nice if output could be written to some sort of intermediate playback format so scenarios can be played back at slower/faster timesteps to better visualize what is happening. With a lot of particles that might not be feasible but maybe a decently high quality video output in the long run.

It would also be nice to color the background/particles based on different fluid properties (e.g., velocity, pressure, vorticity) to better visualize the dynamics. Something like that would be easier to output to video without the particles as well. I'll let you know if I think of anything else as we're creating this. Thanks!