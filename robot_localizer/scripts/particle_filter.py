import rospy
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Quaternion

import tf

import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.patches as patches

from neato import Neato
from map import Map



class ParticleFilter(object):

    class Particle(object):

        def __init__(self, pose_init, weight = 1):
            self.x, self.y, self.theta = pose_init
            self.weight = weight

        def update_pose(self, delta_pose):
            #delta_pose is change in position in robot coordinate frame.

            del_x, del_y, del_theta = delta_pose

            del_global_x = del_x*np.cos(self.theta) + del_y*np.sin(self.theta)

            del_global_y = -del_y*np.cos(self.theta) + del_x*np.sin(self.theta)


            self.x+=del_global_x
            self.y+=del_global_y

            self.theta+=del_theta

        def get_pose(self):
            return (self.x, self.y, self.theta)

        def print_pose(self):
            print(self.get_pose())

        def get_ROS_Pose(self, transform):
            orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
            return Pose(position=Point(x=self.x + transform[0],
                                       y=self.y + transform[1],
                                       z=0),
                        orientation=Quaternion( x=orientation_tuple[0], 
                                                y=orientation_tuple[1], 
                                                z=orientation_tuple[2], 
                                                w=orientation_tuple[3]))




    def __init__(self, n, bounds):
        self.left, self.right, self.top, self.bottom = bounds
        self.certainty = 1;
        self.map_frame = "map" 

        self.particles = []

        width = self.right - self.left
        height = self.bottom - self.top

        for i in range(n):
            rand = np.random.rand(3)
            rand[0] = (rand[0] * width) + self.left
            rand[1] = (rand[1] * height) + self.top
            rand[2] = rand[2] * np.pi
            rand_pose = (rand[0], rand[1], rand[2])
            self.particles.append(self.Particle(rand_pose))

        self.particle_pub = rospy.Publisher("particle_filter_particles", PoseArray, queue_size=10)




    def add_noise(self, k):
        #adds noise from a gaussian distribution with standard deviation given by our certainty and a magnitude k
        
        for particle in self.particles:
            particle.x += np.random.normal(0, self.certainty) * k
            particle.y += np.random.normal(0, self.certainty) * k
            particle.theta += np.random.normal(0, self.certainty) * k
            if (particle.theta > 2*np.pi): particle.theta -= 2*np.pi 
            if (particle.theta < 0): particle.theta += 2*np.pi

    def update_pose_particles(self, delta_pose):
        for part in self.particles:
            part.update_pose(delta_pose)


    def weight_particles(self, map, neato, samples):
        interval = int(360/samples)

        laser_samples = np.zeros(samples)
        index = 0
        for i in range(samples):
            laser_samples[i] = neato.laser_scan_update[index]
            index += interval

        index = 0

        sum_weights = 0 
        for part in self.particles:
            x_pose, y_pose, theta_pose = part.get_pose()
            for laser in laser_samples:



                #print(x_pose, y_pose, theta_pose)



                prob = map.probability_draw((x_pose, y_pose, theta_pose), laser)

                #print(laser, prob)

                theta_pose += interval/1.0 * np.pi/180.0
                theta_pose = theta_pose % (2*np.pi)
                


                part.weight += prob
            sum_weights += part.weight
        self.certainty = 300/(300+sum_weights)






    def resample(self, n):
        new_particles = []

        sum_weights=0
        for part in self.particles:
            sum_weights += part.weight 

        interval = sum_weights/n

        index = np.random.rand() * interval
        start = index

        particle_index = 0;
        new_particle_index = 0;

        while new_particle_index < n:
            if self.particles[particle_index].weight > index:

                new_particles.append(self.Particle(self.particles[particle_index].get_pose()))

                new_particle_index+=1

                index += interval

            else:
                index -= self.particles[particle_index].weight
                particle_index+=1
                if particle_index > n: particle_index -= 1

        self.particles = new_particles
        return (start, interval)



    def publish_part(self, transform):
        particles_ROS_Pose = []
        for part in self.particles:
            particles_ROS_Pose.append(part.get_ROS_Pose(transform))

        self.particle_pub.publish(
            PoseArray(header=Header(stamp=rospy.Time.now(), frame_id=self.map_frame), 
                poses=particles_ROS_Pose))




    def test_resample(self):
        self.particles = []
        weights = [0.1, 0.4, 0.05, 1, 0.02, .5]
        self.certainty = 1/(1+np.sum(weights))
        
        sum_weights = (1/self.certainty) - 1

        part_i = 0

        for i in weights:
            self.particles.append(self.Particle((i,0,0),))
            self.particles[part_i].weight = i
            part_i+=1

        fig = plt.figure()
        ax = fig.add_subplot()

        corner = 0; 
        for i in weights:
            rect = patches.Rectangle((corner, 0), i, 1, facecolor=(i, 0, 0))
            corner += i
            ax.add_patch(rect)

        n = 5

        start, interval = self.resample(n)

        for i in range(n):
            rect = patches.Rectangle((start, 0), 0.01, 1, facecolor=(1, 1, 1))
            start += interval
            ax.add_patch(rect)

        plt.xlim(0, sum_weights)
        ax.set_title("Resample of n=5 from weights [0.1, 0.4, 0.05, 1, 0.02, .5]")


        for i in self.particles:
            i.print_pose

        plt.show()


 



    def test_particle_update(self):

        plt.xlim(-2, 2)
        plt.ylim(-2, 2)
        p = self.Particle((0,0,-.1),)
        plt.scatter(p.x, p.y, color = (1, 0, 0))
        for j in range(19):
            p.update_pose((.1,0, -0.1),)
            p.print_pose()
            #self.add_noise(p, 0.1)
            plt.scatter(p.x, p.y, color = (20/(20+j), 0, 0))
        plt.show()


if __name__ == '__main__':
    rospy.init_node('pf')
    
    neato = Neato()
    world_map = Map()

    bounds = (-world_map.map.info.origin.position.x - 0,
        -world_map.map.info.origin.position.x + 6,
        -world_map.map.info.origin.position.y - 1,
        -world_map.map.info.origin.position.y + 4)





    pf = ParticleFilter(1000, bounds)


   
    r = rospy.Rate(3)

    
    while not rospy.is_shutdown():

        neato.update()
        if (neato.delta_pose is not None) : pf.update_pose_particles(neato.delta_pose)
        if neato.laser_scan_update is not None :
            pf.add_noise(0.2)
            pf.weight_particles(world_map, neato, 20)
            pf.resample(1000)
            print(pf.certainty)
        
        pf.publish_part((world_map.map.info.origin.position.x, world_map.map.info.origin.position.y),)
        r.sleep()
    
        