<Please submit this file with your solution.>

CSCI 520, Assignment 1

Airi Pham

================

<Description of what you have accomplished>
Base criteria:
Animate the movement of a jello cube based on a realistic physical model.
Must incorporate a bounding box, including collision detection and response.
Must implement an arbitrary non-homogeneous time-independent force field, with appropriate force-field interpolation.
Use the provided code to interactively position the camera, use proper lighting, and render the cube in wireframe and triangle mode.
Read the description of the world from a world file and simulate the cube according to this information. Your program should work with arbitrary world files. Your program, of course, need not work correctly for world files containing invalid data, bad integrator parameters (blow up effects and similar), bad elasticity, damping parameters and other physical parameters, or bad position or velocity parameters.
Run at interactive frame rates (>15fps at 640x480)

<Also, explain any extra credit that you have implemented.>
- Textured cube with hand-drawn texture image
- Inclined plane collision logic & render
- Customized lighting and material
- Optimized conversion from float to int and avoid float division in external force interpolation
- Interactivity with the cube (drag left mouse anywhere on the screen and the force will apply to the cube)