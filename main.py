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
from agent_dwa import dwaAgent

# Globals
show_animation = True

def main():
    print(__file__ + " start!!")
    # initial state [x, y, yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([5, 5, math.pi / 8.0, 0.0, 0.0])
    # goal position [x, y]
    goal = np.array([50, 50])
    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt("world01.csv")

    traj = np.array(x)
    ticks = 0

    u = np.array([0.0, 0.0])
    dwa = dwaAgent()

    for i in range(1000):
        u, ltraj = dwa.dwa_control(x, u, goal, np.array([ob[:, 0], ob[:, 1]]))
        x = dwa.motion(x, u)
        traj = np.vstack((traj, x))  # store state history

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            dwa.plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= dwa.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.show()


if __name__ == '__main__':
    main()
