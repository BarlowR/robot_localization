
import rospy
from nav_msgs.srv import GetMap


import numpy as np 
import matplotlib.pyplot as plt 



class Map(object):
    # Attributes;
    #   map : given ROS OccupancyGrid of the area 
    #   likelihood_field : 2D array representing the likelyhood field of the area 

    def __init__(self):
        
        #pull map from server
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map

        #extract map occupancy data as 2d array
        map_array = np.array(self.map.data).reshape((self.map.info.width, self.map.info.height))
        map_array_no_neg = map_array + 1
        self.likelihood_field = np.zeros((map_array.shape))
        
        #convolve map with gaussian-like kernel to get a laser scan likelihood field, and normalize the resulting array. The result is padded with zeros around the edges to maitain an array with the same size as the original
        kernel_px_size = 11
        likelihood_field_offset = int((kernel_px_size-1)/2)
        kernel = self.gaussian(kernel_px_size, .5,.2)
        
        convolved = self.convolve2D(map_array_no_neg, kernel)
        con_max = np.amax(convolved)
        convolved = convolved/con_max  #This normalizes the array to be in the range (0,1)

        end_index_x = convolved.shape[0]+likelihood_field_offset
        end_index_y = convolved.shape[1]+likelihood_field_offset

        self.likelihood_field[likelihood_field_offset:end_index_x, likelihood_field_offset:end_index_y] = convolved # Inserts the convolved array into an array of zeros in the correct position so that it lines back up with the map coordinate frame

            


    
    def probability_draw(self, pose, dist):
        # pose is x, y in the map coordinate frame & units and ang in radians, with 0 pointing to the right and pi/2 pointing down
        # dist is the laser range distance in meters. Inf is a valid arguement. 

        if (dist == "Inf"):
            return 0.4
        if (dist < 0.1):
            return 0
        else:
            x, y, ang = pose
            x_pose_pixels = int(x/self.map.info.resolution)
            y_pose_pixels = int(y/self.map.info.resolution)

            x_scan = (dist*np.cos(ang)) + x
            y_scan = (dist*np.sin(ang)) + y

            x_scan_pixels = int(x_scan/self.map.info.resolution)
            y_scan_pixels = int(y_scan/self.map.info.resolution)


            '''
            plot pose as red and scan point as green
            plt.imshow(self.likelihood_field, cmap="gray")
            plt.scatter(x_pose_pixels, y_pose_pixels, color="red")
            plt.scatter(x_scan_pixels, y_scan_pixels, color="green")
            plt.show()
            '''
            if (y_scan_pixels < self.map.info.width and x_scan_pixels < self.map.info.height):
                return self.likelihood_field[y_scan_pixels][x_scan_pixels]
            else: return 0


    def plot_probability_draw(self, pose):

        x, y, ang = pose
        x_pose_pixels = int(x/self.map.info.resolution)
        y_pose_pixels = int(y/self.map.info.resolution)
  
        f, axarr = plt.subplots(2,1, gridspec_kw={'height_ratios': [5, 1]})

        axarr[0].imshow(self.likelihood_field, cmap="gray")
        axarr[0].scatter(x_pose_pixels, y_pose_pixels, color="red")
  
        for i in np.arange(0.2, 6, 0.01):
            axarr[1].scatter(i, self.probability_draw((x, y, ang), i), s = 0.3, color = (1,0,0,1))

            x_scan = (i*np.cos(ang)) + x
            y_scan = (i*np.sin(ang)) + y

            x_scan_pixels = int(x_scan/self.map.info.resolution)
            y_scan_pixels = int(y_scan/self.map.info.resolution)

            axarr[0].scatter(x_scan_pixels, y_scan_pixels, s = 2, color=(1,0,0,0.03))

        axarr[0].invert_yaxis()
        plt.show()


    def convolve2D(self, image, kernel): #kernel should be square w/ odd number of rows. Both args should be numpy arrays 
        if (kernel.shape[0] == kernel.shape[1] and kernel.shape[0]%2==1):
            x = image.shape[0]-kernel.shape[0]+1
            y = image.shape[1]-kernel.shape[0]+1
            output = np.zeros((x,y))
            for i in range(x):
                for j in range(y):
                    output[i][j] = np.sum(image[i:(i+kernel.shape[0]), j:(j+kernel.shape[0])]*kernel)
            return output
        else:
            print("kernel sizing error")
            return image

    def gaussian(self, side_length, sigma, mu):
        if (side_length%2==1):
            x, y = np.meshgrid(np.linspace(-1,1,side_length), np.linspace(-1,1,side_length))
            d = np.sqrt(x*x+y*y)
            g = np.exp(-( (d-mu)**2 / ( 2.0 * sigma**2 ) ) )
            return g

    def plot_gaussian(self):
        f, axarr = plt.subplots(3,3)
        i_ind = 0
        j_ind = 0
        for sig in [1,.7,.5]:
            for mu in [0, 0, 0]:
                axarr[i_ind,j_ind].imshow(self.gaussian(31, sig, mu), cmap="gray")
                title = "sigma = " + str(sig) + ", mu = " + str(mu)
                axarr[i_ind,j_ind].set_title(title)
                j_ind+=1
            j_ind = 0
            i_ind+=1
        plt.show()


            


if __name__ == '__main__':
    m = Map()
    m.plot_probability_draw((16.289163063732026, 3.6998839364401728, 0.4136842711695055),)