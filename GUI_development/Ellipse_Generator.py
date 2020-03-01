# Ellipse_Generator.py
# @author: Austin Shin

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from scipy import optimize

class Ellipse_Generator:
    """
    Generates circle or ellipse based on user-clicked points on grid
    """

    def __init__(self):
        """
        Initializes grid points and displays plot of it.

        Args:
            None

        Returns:
            None
        """
        # blue points used in circle fit
        self.x = []
        self.y = []

        # Initialize coordinates of grid points
        self.grid_x = []
        self.grid_y = []
        self.bool_grid = {}
        for i in range(-19, 20, 2):
            for j in range(-19, 20, 2):
                self.grid_x.append(i)
                self.grid_y.append(j)
                self.bool_grid[(i, j)] = False

        # plot grid
        fig = plt.figure()
        self.ax = fig.add_subplot(111)
        plt.scatter(self.grid_x, self.grid_y, s=4, facecolor='0.4', marker='s')
        plt.gray()
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        click_conn_id = fig.canvas.mpl_connect('button_press_event', \
            self.onclick_callback)

        # button to generate circle fit
        ax_button1 = plt.axes([0.15, 0.03, 0.2, 0.05])
        gen_button1 = Button(ax_button1, 'Generate Circle')
        gen_button1.on_clicked(self.generate_circle)

        # button to generate ellipse fit
        ax_button2 = plt.axes([0.4, 0.03, 0.2, 0.05])
        gen_button2 = Button(ax_button2, 'Generate Ellipse')
        gen_button2.on_clicked(self.generate_ellipse)

        # button to clear blue points
        ax_button3 = plt.axes([0.65, 0.03, 0.2, 0.05])
        gen_button3 = Button(ax_button3, 'Clear')
        gen_button3.on_clicked(self.clear_grid)

        self.ax.axis('equal')
        plt.show()

    @staticmethod
    def find_closest_odd(x):
        """
        Finds closest odd number to x

        Args:
            x: number to round to closest odd number

        Returns:
            closest odd number
        """
        floor_x = math.floor(x)
        if floor_x % 2 == 0:
            return floor_x + 1
        return floor_x

    def calc_R(self, xc, yc):
        """
        Adapted from: https://scipy-cookbook.readthedocs.io/items/Least_Squares_Circle.html
        Calculate the distance of each 2D points from the center (xc, yc).

        Args:
            xc: x coordinate of center
            yc: y coordinate of center

        Returns:
            array of distances of each point from center 
        """
        return np.sqrt((self.x-xc)**2 + (self.y-yc)**2)

    def f_2(self, c):
        """
        Adapted from: https://scipy-cookbook.readthedocs.io/items/Least_Squares_Circle.html
        Calculate the algebraic distance between the data points and the mean 
        circle centered at c=(xc, yc).

        Args:
            c: center coordinate

        Returns:
            array of residuals of distance of points from center
        """
        Ri = self.calc_R(*c)
        return Ri - Ri.mean()

    def onclick_callback(self, event):
        """
        Callback method when user clicks somewhere on plot. Turns gray point to
        blue point and vice versa if user clicks relatively close to it.

        Args:
            event: mouse click

        Returns:
            None
        """
        if event.inaxes==self.ax:
            x, y = event.xdata, event.ydata # center of circle
            closest_odd_x = self.find_closest_odd(x)
            closest_odd_y = self.find_closest_odd(y)

            # check if user click location is close to grid point
            if abs(round(x) - closest_odd_x) < 0.2 and \
                    abs(round(y) - closest_odd_y) < 0.2:
                grid_x = round(x)
                grid_y = round(y)
                if self.bool_grid[(grid_x, grid_y)]: # blue to gray
                    self.ax.scatter(grid_x, grid_y, s=4, facecolor='0.4',
                        marker='s')
                    plt.gray()
                    self.bool_grid[(grid_x, grid_y)] = False
                else: # gray to blue
                    self.ax.scatter(grid_x, grid_y, color='blue', s=4,
                        marker='s')
                    self.bool_grid[(grid_x, grid_y)] = True

                self.ax.axis('equal')
                plt.pause(0.001)
                plt.draw()

    def generate_circle(self, event):
        """
        Plots circle that best fits blue points.

        Args:
            event: click on generate circle button

        Returns:
            None
        """
        self.ax.clear()

        # draw grid of gray points
        self.ax.scatter(self.grid_x, self.grid_y, s=4, facecolor='0.4', 
            arker='s')
        plt.gray()
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        blue_pts = []
        for (key, value) in self.bool_grid.items():
            if value:
                blue_pts.append([key[0], key[1]])
        blue_pts = np.asarray(blue_pts)

        if len(blue_pts) != 0:
            center = np.mean(blue_pts, axis=0)

            # draw blue points
            self.ax.scatter(blue_pts[:,0], blue_pts[:,1], color='blue', s=4, \
                marker='s')

        if len(blue_pts) == 2: # let 2 blue points define diameter of circle
            dist = 0
            for pt in blue_pts:
                dist += math.sqrt( (pt[0]-center[0])**2 + (pt[1]-center[1])**2 )
            r = dist / len(blue_pts)

            # draw blue circle
            blue_circ = plt.Circle(tuple(center), r, color='b', fill=False)
            self.ax.add_artist(blue_circ)
        elif len(blue_pts) == 3:
            # adapted from: https://www.geeksforgeeks.org/equation-of-circle-when-three-points-on-the-circle-are-given/
            x12 = blue_pts[0,0] - blue_pts[1,0] 
            x13 = blue_pts[0,0] - blue_pts[2,0] 

            y12 = blue_pts[0,1] - blue_pts[1,1]
            y13 = blue_pts[0,1] - blue_pts[2,1]

            y31 = blue_pts[2,1] - blue_pts[0,1]
            y21 = blue_pts[1,1] - blue_pts[0,1]

            x31 = blue_pts[2,0] - blue_pts[0,0] 
            x21 = blue_pts[1,0] - blue_pts[0,0] 

            sx13 = pow(blue_pts[0,0], 2) - pow(blue_pts[2,0], 2)

            sy13 = pow(blue_pts[0,1], 2) - pow(blue_pts[2,1], 2)

            sx21 = pow(blue_pts[1,0], 2) - pow(blue_pts[0,0], 2)
            sy21 = pow(blue_pts[1,1], 2) - pow(blue_pts[0,1], 2)

            f = (((sx13) * (x12) + (sy13) * 
                (x12) + (sx21) * (x13) + 
                (sy21) * (x13)) // (2 * 
                ((y31) * (x12) - (y21) * (x13))))
     
            g = (((sx13) * (y12) + (sy13) * (y12) + 
                (sx21) * (y13) + (sy21) * (y13)) // 
                (2 * ((x31) * (y12) - (x21) * (y13))))

            c = (-pow(blue_pts[0,0], 2) - pow(blue_pts[0,1], 2) - 
                2 * g * blue_pts[0,0] - 2 * f * blue_pts[0,1])

            h = -g
            k = -f
            sqr_of_r = h * h + k * k - c 
            r = math.sqrt(sqr_of_r)

            # draw blue circle
            blue_circ = plt.Circle((h,k), r, color='b', fill=False)
            self.ax.add_artist(blue_circ)
        elif len(blue_pts) > 3: # uses least squares to calc circle center
            # Adapted from: https://scipy-cookbook.readthedocs.io/items/Least_Squares_Circle.html
            self.x = np.r_[blue_pts[:,0]]
            self.y = np.r_[blue_pts[:,1]]
            center_lsq, _ = optimize.leastsq(self.f_2, tuple(center))
            ri = self.calc_R(*center_lsq)
            r = ri.mean()

            # draw blue circle
            blue_circ = plt.Circle(tuple(center), r, color='b', fill=False)
            self.ax.add_artist(blue_circ)
        else:
            print("Circle can only be generated when there are at least 2 blue points.")

        self.ax.axis('equal')
        plt.pause(0.001)
        plt.draw()

    def generate_ellipse(self, event):
        """
        Adapted from: https://stackoverflow.com/questions/47873759/how-to-fit-a-2d-ellipse-to-given-points
        Plots ellipse that best fits blue points.

        Args:
            event: click on generate ellipse button

        Returns:
            None
        """
        self.ax.clear()

        # draw grid of gray points
        self.ax.scatter(self.grid_x, self.grid_y, s=4, facecolor='0.4',
            marker='s')
        plt.gray()
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        blue_pts = []
        for (key, value) in self.bool_grid.items():
            if value:
                blue_pts.append([key[0], key[1]])
        blue_pts = np.asarray(blue_pts)
        center = np.mean(blue_pts, axis=0)

        # draw blue points
        self.ax.scatter(blue_pts[:,0], blue_pts[:,1], color='blue', s=4, \
            marker='s')

        # uses SVD to find transformation from circle to ellipse
        if len(blue_pts) >= 5:
            x = blue_pts[:,0] - center[0]
            y = blue_pts[:,1] - center[1]
            U, S, V = np.linalg.svd(np.stack((x,y)))

            # draw blue circle
            tt = np.linspace(0, 2*np.pi, 1000)
            circle = np.stack((np.cos(tt), np.sin(tt)))
            transform = np.sqrt(2/len(blue_pts)) * U.dot(np.diag(S))
            fit = transform.dot(circle) + np.array([[center[0]], [center[1]]])
            self.ax.plot(fit[0, :], fit[1, :], 'b')
        else:
            print("Ellipse can only be generated when there are at least 5 blue points.")

        self.ax.axis('equal')
        plt.pause(0.001)
        plt.draw()

    def clear_grid(self, event):
        """
        Change all blue points to gray points, and clear any existing circle or
        ellipse.

        Args:
            event: mouse click on Clear button

        Returns:
            None
        """
        self.ax.clear()

        # draw grid of gray points
        self.ax.scatter(self.grid_x, self.grid_y, s=4, facecolor='0.4',
            marker='s')
        plt.gray()
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        self.ax.axis('equal')
        plt.pause(0.001)
        plt.draw()

        for (key, _) in self.bool_grid.items():
            self.bool_grid[key] = False

gen = Ellipse_Generator()
