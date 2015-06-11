# path
c++ library for calculating and displaying motion

Path class is a container for motion data. 
it stores position in a std::vector
it can save/load from position data in a file
it also stores a timestamp for each position, and has an optimization for fixed-step timestamps
it can calculate velocities between positions, and extrapolate points for timestamps with no measured data
it can combine its data with another Path's data, and create compound Paths

here is a modified bgfx example 13 using Path to displaymotion in the cubes. The path the cubes take is the combination of two circles with different radii and speed:
http://gfycat.com/ValuablePowerlessAngora
