"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
	# https://docs.scipy.org/doc/numpy/user/basics.creation.html
import matplotlib.pyplot as plt
	# https://matplotlib.org/users/pyplot_tutorial.html

# Modules
from agent_random import rAgent

# Globals
show_animation = True

def main():
    print(__file__ + " start!!")
    # initial state [x, y]
    x = np.array([10, 10])
    # goal position [x, y]
    goal = np.array([50, 50])
    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt("myworld.csv")

    traj = np.array(x)
    ticks = 0

    ra = rAgent()

    for i in range(1000):
        if (ticks == 0):
            ang, vel, ticks = ra.random_control()
        else:
            ticks-=1

        x,col = ra.motion(x, ob, ang, vel)
        if (col):
            ticks = 0
        traj = np.vstack((traj, x))  # store state history

        if show_animation:
            plt.cla()
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            # ob[:, 0] -> The full first row of the array (all X numbers)
            # ob[:, 1] -> The full second row of the array (all Y numbers)
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= ra.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.show()


if __name__ == '__main__':
    main()
