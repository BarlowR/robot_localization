## Implementation Plan

### Overview & Resources

As the second project for the Olin course, Introduction to Computational Robotics, I plan to implement a particle filter to localize a Neato robot inside of a given map using lidar data. The particle filter will be implemented as described in Section 3, "The basic particle filter," of Kunsch et. all, 2013. I plan to write this program without scaffolding. 

### Extensions

If I have time to work beyond the basic particle filter, my first extension will be the implementation of a balanced resampling scheme and the omission of resampling when weights are sufficiently uniform. Section 3.4 of Kunsch et. all describes these improvements in further detail. After this, I plan to create a metric of computation time availability and write a function that dynamically alters the number of particle samples based on this metric, to ensure that this filter can be used to the limit of its precision within the computational bottleneck.


### Filtering Cycle Steps

1. Create N new particles over the weighted distribution from the previous cycle, each with weight 1/N. If this is the first cycle, initialize the particles randomly. 
    (TODO: Determine a resampling strategy https://www.researchgate.net/publication/274404127_Resampling_Methods_for_Particle_Filtering_Classification_implementation_and_strategies/link/5598db5608ae793d137e21ff/download)
2. Compare the sensor data from the Neato to the particle, and give each particle a weight based on the probability of the sensor data matching. 
3. Propagate forward the particles using the odometry data from the Neato. Add noise from a random draw from a Gaussian distribution.
4. Select the particle with the highest weight to represent the pose of the Neato.


### Required Objects

#### Map
##### Overview
Handles the provided environmental map and provides helper functions for mocking laser scans
##### Methods
###### init (OccupancyGrid map_file)
map_file is the representation of the map in ROS's nav_msgs/OccupancyGrid.msg

###### dist_to_wall (Tuple pose)
pose is 3 Float values, which correspond to (x, y, ang) where x and y are position in map units and ang is angle in radians, with 0 pointing up and pi/2 pointing to the right.

returns a single Float value corresponding to the distance in meters to the closest occupancy pixel with probability of occupancy over 75%. 
returns Inf if over 10m. 

#### Neato
##### Overview
Subscribes to neato topics and maintains a short history of sensor & odom readings. 
class variables: 
##### Methods
###### init ()

###### update

#### Particle
##### Overview
A particle object for the particle filter 
class variables: 
##### Methods
###### init(


Kunsch et. all, 2013. https://arxiv.org/pdf/1309.7807.pdf
