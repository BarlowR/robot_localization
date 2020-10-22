# The Particle Filter

## Overview

As the second project for Olin's Computational Robotics, I implemented a particle filter to locate a Neato robot inside of a given environment using laser scan data. Below is a gif of the particle filter converging; each small red arrow is a particle, and the large red arrow is the actual pose of the robot. 

<img src = "https://github.com/BarlowR/robot_localization/blob/master/robot_localizer/working.gif">



## Implementation

My initial implementation plan can be found [here](https://github.com/BarlowR/robot_localization/blob/master/Implementation_Plan.md). I've had to make some changes to the structure of the cprogram while writing the methods, but the basic structure is there. There is a Map class which handles the environment, a Neato class which handles the information published by Neato, and a ParticleFilter class which creates the particle filter and handles its operation.

### Map

At startup, a Map object pulls the current OccupancyGrid from the ROS server, and stores a copy locally. Next, the Map takes the occupancy grid data from ROS message and convolves it with a gaussian-like kernel to create a likelihood field representation of the local environment. Below is a grid of different gausian kernels that I created as a visualization while evaluating which one to use, and an example of a liklihood field next to the original occupancy grid.

| Kernel Grid | Liklihood Field | 
| ------- | ----- |
| <img src="https://github.com/BarlowR/robot_localization/blob/master/Gaussian%20Kernels.png" alt="Kernel" width = "500">  |  <img src="https://github.com/BarlowR/robot_localization/blob/master/Example%20Likelihood%20Field.png" alt="Likelihood Field" width = "400"> |


#### probability_draw(pose, dist)
Within the Map class, the probability_draw function takes a pose Tuple (x,y,theta) and a laser scan distance, and returns the likelihood of that scan distance given that specific pose. To illustrate this function, below is an image of an example pose within a sample map, with a simulated laser scan in red. The plot below shows the likelihood of  given scan on the y axis, with scan distance on the x axis. 

<img src = "https://github.com/BarlowR/robot_localization/blob/master/Likelihood%20Field%20Implementation.png">

### Neato

A Neato object subscribes to the Neato robot laser scans and transforms, and maintains time specific information required to operate the particle filter effectively. Specifically, it makes available the Neato's motion and laser scans at discete time steps determined by when the update function was called last. This allows the particle filter to call the update function, reference the data, spend time in computation, and then repeat at whatever frequency is required without measurements/calculations shifting due to updated laser scans or robot motion. 

#### update()
Computes the transform of the neato robot since the last time the update function was called, and saves a copy of the latest laser scan.


### ParticleFilter (n, bounds)

Now we get to the good part. The particle filter takes information from instances of the map and the neato classes, and computes an estimated pose of the neato in the map environment based on laser scan data.

When a ParticleFileter object is initialized, it creates n Particles inside of given bounds (see Particle class below). The only attribute of note is the certainty attribute, which is inversely proportional to the sum of all of the weights of the particles in the system, such that a value of 1 is complete uncertainty and 0 is absolute certainty. 

#### (Class) Particle

The Particle class represents a single particle. It has a pose and a weight, and includes a few helper functions for getting the pose of the particle and transforming it into ROS Pose format.

#### add_noise(k)

This function iterates through all the particles and adds random noise to the pose, drawn from a gausian distribution with a standard deviation equal to the ParticleFilter Object's certainty. This means that as the particle filter is more sure of its state estimation, less and less noise is added and the system can converge closer.

#### weight_particles(map, neato, samples)

This function takes in references to a map object and a neato object, and for each particle, it computes the weight of the particles by taking the product of all of the likelihoods of n=samples laser scans using the Map's probability_draw() function.


#### resample(n)

This functiton draws n new particles from the current weighted distibution of particles. To visualize how this is done, imagine a wagon wheel with all particles distributed along the outside of the wheel. The arc length of each particle is given by its weight. To resample, a set of n equally spaced spokes are created with a random orientation, and the new particles are selected from the old set by which arcs are pointed to by each spokes. I wasn't able to create a good visualization of this using MatPlotlib, but one could imagine the below rectangle projected along the outside of a circle, with the white lines representing the "spokes," or particles that are selected. Note that the shade of rec of each particle's rectangle is given by its weight.

<img src = "https://github.com/BarlowR/robot_localization/blob/master/Resampling.png">



## Running the Particle Filter

First, the Map and Neato instances are initalized, then the ParticleFilter instance.

The main loop of this program is quite simple; at each timestep, the following is excecuted:
* The Neato is polled for the latest movement with Neato.update(), and the particles are updated accordingly with ParticleFilter.update_pose_particles()
* Noise is added to each particle with ParticleFilter.add_noise()
* The weight of each particle is calculated with PartifleFilter.weight_particles()
* The particles are resampled with ParticleFilter.resample()
* Finally, all particles are published as a PoseArray for visualization with RViz.

## Final Notes

### Challenges
I had a bit of trouble initially working out the transforms between the map frame, the particles and the neato. I think this was in part due to me initially defining theta = 0 to be pointing from the origin to (0,1). After I went back and fixed that convention to theta=0 points to (1,0) everywhere in my code, I had a lot less trouble fixing the tranformation bugs.

### Future Improvements

There seem to be a lot of knobs to turn in this particle filter, from number of particle to resampling rate to noise magnitude, and on and on. I'd like to spend more time playing with the implementation to determine the effect of each one, and how they can work together to create a faster/more robust system.  I'd also still like to implement a way to dynamically alter the number of particles in the filter; I have the ability to resample any number of particles, but I didn't have the time to determine how best to control this number.
