import rospy
from std_msgs.msg import Header, String
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Quaternion

import tf

import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.patches as patches



class ParticleFilter(object):

    class Particle(object):

        def __init__(self, pose_init, weight = 0):
            self.x, self.y, self.theta = pose_init
            self.weight = weight

        def update_pose(self, delta_pose):
            #delta_pose is change in position in robot coordinate frame.

            del_x, del_y, del_theta = delta_pose

            del_global_x = del_y*np.sin(self.theta) + del_x*np.cos(self.theta)

            del_global_y = del_x*np.sin(self.theta) + del_y*np.cos(self.theta)


            self.x+=del_global_x
            self.y+=del_global_y

            self.theta+=del_theta

        def get_pose(self):
            return (self.x, self.y, self.theta)

        def print_pose(self):
            print(self.get_pose())

        def get_ROS_Pose(self):
            orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
            return Pose(position=Point(x=self.x,
                                       y=self.y,
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




    def add_noise(self, particle, k):
        #adds noise from a gaussian distribution with standard deviation given by our certainty and a magnitude k
        particle.x += np.random.normal(0, self.certainty) * k
        particle.y += np.random.normal(0, self.certainty) * k
        particle.theta += np.random.normal(0, self.certainty) * k
        if (particle.theta > 2*np.pi): particle.theta -= 2*np.pi 
        if (particle.theta < 0): particle.theta += 2*np.pi

    def resample(self, n):
        new_particles = []

        sum_weights = (1/self.certainty) - 1 #this should be equal to the sum of all of the particle weights

        interval = sum_weights/n

        index = np.random.rand() * interval
        start = index

        particle_index = 0;
        new_particle_index = 0;

        while new_particle_index < n:
            if self.particles[particle_index].weight > index:

                new_particles.append(self.Particle(self.particles[particle_index].get_pose()))

                new_particles[new_particle_index].print_pose()


                new_particle_index+=1

                index += interval

            else:
                index -= self.particles[particle_index].weight
                particle_index+=1

        self.particles = new_particles
        return (start, interval)

    def publish_part(self):
        particles_ROS_Pose = []
        for part in self.particles:
            particles_ROS_Pose.append(part.get_ROS_Pose())

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
        plt.ylim(0, 2.5)
        p = self.Particle((0,0,0),)
        plt.scatter(p.x, p.y, color = (1, 0, 0))
        for j in range(19):
            p.update_pose((0,.1, 0),)
            self.add_noise(p, 0.1)
            plt.scatter(p.x, p.y, color = (20/(20+j), 0, 0))
        plt.show()


if __name__ == '__main__':
    rospy.init_node('pf')
    pf = ParticleFilter(400, (0, 10, 0, 10))
    r = rospy.Rate(5)


    while not rospy.is_shutdown():
        pf.publish_part()
        r.sleep()
