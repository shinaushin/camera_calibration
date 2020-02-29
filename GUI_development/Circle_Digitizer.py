import math
import numpy as np
import matplotlib.pyplot as plt

class Circle_Digitizer:
    """
    Digitizes circle based on a grid containing 20x20 points.
    """

    def __init__(self):
        """
        Initializes grid points and displays plot of it.

        Args:
            None

        Returns:
            None
        """
        self.cx = 0
        self.cy = 0

        # Initialize coordinates of grid points
        self.grid_x = []
        self.grid_y = []
        for i in range(-19, 20, 2):
            for j in range(-19, 20, 2):
                self.grid_x.append(i)
                self.grid_y.append(j)

        # plot grid
        fig = plt.figure()
        self.ax = fig.add_subplot(111)
        plt.scatter(self.grid_x, self.grid_y, s=4, facecolor='0.4', marker='s')
        plt.gray()
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        click_conn_id = fig.canvas.mpl_connect('button_press_event', \
            self.onclick_callback)
        release_conn_id = fig.canvas.mpl_connect('button_release_event', \
            self.release_callback)

        plt.axis('equal')
        plt.show()

        print("Click to place the center of a circle, and drag to set its radius.")

    def onclick_callback(self, event):
        """
        Callback method when user clicks somewhere on plot.

        Args:
            event: mouse click

        Returns:
            None
        """
        self.cx, self.cy = event.xdata, event.ydata # center of circle

        # clear axes and re-plot grid
        self.ax.clear()
        plt.scatter(self.grid_x, self.grid_y, s=4, facecolor='0.4', marker='s')
        plt.gray()
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        plt.axis('equal')
        plt.pause(0.001)
        plt.draw()

    def release_callback(self, event):
        """
        Callback method when user releases mouse click on plot.

        Args:
            event: mouse click release

        Returns:
            None
        """
        px, py = event.xdata, event.ydata # point on circle

        if px != None and py != None: # if point lies in grid
            r = math.sqrt( (px-self.cx)**2 + (py-self.cy)**2 )
            
            # find lower, upper bounds of points to search through
            upper_bound_x = math.ceil(self.cx + r)
            lower_bound_x = math.floor(self.cx - r)
            upper_bound_y = math.ceil(self.cy + r)
            lower_bound_y = math.floor(self.cy - r)

            if upper_bound_x <= 19 and upper_bound_y <= 19 and \
                    lower_bound_x >= -19 and lower_bound_y >= -19:
                blue_circ_pts = []
                radii = []
                # filter grid points closest to circle
                for i in range(lower_bound_x, upper_bound_x+1):
                    for j in range(lower_bound_y, upper_bound_y+1):
                        if i % 2 == 1 and j % 2 == 1:
                            dist = math.sqrt( (self.cx-i)**2 + (self.cy-j)**2 )
                            if abs(dist-r) <= 1:
                                radii.append(dist)
                                blue_circ_pts.append([i,j])

                if len(blue_circ_pts) > 0:
                    # plot circle defined by user mouse clicks
                    blue_circ_pts = np.asarray(blue_circ_pts)
                    plt.scatter(blue_circ_pts[:,0], blue_circ_pts[:,1], \
                        color='blue', s=4, marker='s')

                    if len(blue_circ_pts) > 1:
                        outer_circ_r = max(radii) # radius of outer red circle
                        inner_circ_r = min(radii) # radius of inner red circle
                        # if there exists a blue point outside blue circle
                        if outer_circ_r > r:
                            red_circ1 = plt.Circle((self.cx, self.cy), \
                                outer_circ_r, color='r', fill=False)
                            self.ax.add_artist(red_circ1)
                        else:
                            print("Outer circle could not be constructed because some part of it lies outside the grid.")
                        # if there exists a blut point inside blue circle
                        if inner_circ_r < r:
                            red_circ2 = plt.Circle((self.cx, self.cy), \
                                inner_circ_r, color='r', fill=False)
                            self.ax.add_artist(red_circ2)
                        else:
                            print("Inner circle could not be constructed because some part of it lies outside the grid.")
                    else:
                        print("Not enough blue points to construct red circles.")
                else:
                    print("Not enough blue points to construct red circles.")

                # draw blue circle
                blue_circ = plt.Circle((self.cx, self.cy), r, color='b', fill=False)
                self.ax.add_artist(blue_circ)

                plt.axis('equal')
                plt.pause(0.001)
                plt.draw()
            else:
                print("Circle too big to fit inside the grid.")
        else:
            print("Please release mouse click while inside the grid of points.")

dig = Circle_Digitizer()
