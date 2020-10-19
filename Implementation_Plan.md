# Implementation Plan

## Overview & Resources

As the second project for the Olin course, Introduction to Computational Robotics, I plan to implement a particle filter to localize a Neato robot inside of a given map using lidar data. The particle filter will be implemented as described in Section 3, "The basic particle filter," of Kunsch et. all, 2013. I plan to write this program without scaffolding. 

## Extensions

If I have time to work beyond the basic particle filter, my first extension will be the implementation of a balanced resampling scheme and the omission of resampling when weights are sufficiently uniform. Section 3.4 of Kunsch et. all describes these improvements in further detail. After this, I plan to create a metric of computation time availability and write a function that dynamically alters the number of particle samples based on this metric, to ensure that this filter can be used to the limit of its precision within the computational bottleneck.


## Filtering Cycle Steps

1. Create N new particles over the weighted distribution from the previous cycle, each with weight 1/N. If this is the first cycle, initialize the particles randomly. 
    (TODO: Determine a resampling strategy https://www.researchgate.net/publication/274404127_Resampling_Methods_for_Particle_Filtering_Classification_implementation_and_strategies/link/5598db5608ae793d137e21ff/download)
2. Compare the sensor data from the Neato to the particle, and give each particle a weight based on the probability of the sensor data matching. 
3. Propagate forward the particles using the odometry data from the Neato. Add noise from a random draw from a Gaussian distribution.
4. Select the particle with the highest weight to represent the pose of the Neato.


## Objects

### Map
#### Overview
Handles the provided environmental map and provides functionality for making a likelihood map & drawing values from that map
#### Attributes
**OccupancyGrid map:** The given map of the area 

**OccupancyGrid likelihood_field:** The likelyhood field of the area 

#### Methods
##### init (OccupancyGrid map_file)
* map_file is the representation of the map in ROS's nav_msgs/OccupancyGrid.msg
* This function creates the likelihood field by convolving the map occupancy image with a gaussian kernel. 

##### probability_draw (Tuple pose, Float32 dist)
* pose is 3 Float values, which correspond to (x, y, ang) where x and y are position in map units and ang is angle in radians, with 0 pointing up and pi/2 pointing to the right. 
* dist is a laser range value. 
* returns a single Float value corresponding to the probability of the laser range arguement given the pose arguement & occupancy field. See image below:

<img src = "https://github.com/BarlowR/robot_localization/blob/master/Likelihood%20Field.png" width = "500">



### Neato
#### Overview
Subscribes to neato topics and maintains a short history of sensor & odom readings. 

#### Attributes
#### Methods
##### init ()
##### update ()



### ParticleFilter
#### Overview
The particle filter

#### Attributes
**Neato neat:** A neato robot object 
**Map m:** A map object
**Float certainty:** A measure of the certainty of the particle filter. Begins at 1, and decreases with higher certainty according to 1/(1 + Σ (particle weights)) 
**Particle[] particles:** A list containing all of the particles within the filter. 
**Particle best_estimation:** The particle with the highest positional certainty

#### Subclasses

#### Particle 
##### Overview
A particle object
##### Attributes
**Float[] pose [x, y, theta]:** The particle's position in map units & coordinate frame.
**Float weight:** The weight of the particle. 0 is complete uncertainty, 1 is absolute certainty.

##### Methods
###### initialize(Tuple pose_init)
* Tuple pose_init: the pose at which to initialize the particle
* Initializes a new particle at pose_init

###### update_pose(Tuple delta_pose)
* Tuple delta_pose: changes in neato pose (x, y and theta) from the baselink coordinate system.
* Updates the pose property based on the delta_pose arguement  

###### random_noise()
* Adds random noise to pose from a gaussian distribution with magnitude determined by the certainty property of the ParticleFilter


#### Methods
##### init(Int n, Tuple bounds)
* Int n: number of particles to initialize
* Tuple bounds: (x1, x2, y1, y2) distances from the origin to the right, left, top and bottom bounds of the rectangle in qhich to initialize particles
* Initializes n particles into the particles array attribute

##### resample(Int n)
* Int n: number of particles to resample
* Resamples n new particles from the current set of particles using a [stochastic universal sampling method](https://www.youtube.com/watch?v=tvNPidFMY20)


Kunsch et. all, 2013. https://arxiv.org/pdf/1309.7807.pdf
