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
    x = np.array([10, 10, math.pi / 8.0, 0.0, 0.0])
    # goal position [x, y]
    goal = np.array([50, 50])
    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt("world03.csv")

    traj = np.array(x)
    ticks = 0

    u = np.array([0.0, 0.0])
    dwa = dwaAgent()

    for i in range(1000):
        u, ltraj = dwa.dwa_control(x, u, goal, ob)
        x = dwa.motion(x, u)
        print "Pos " + str(i) + " - x: " + str(x)
        traj = np.vstack((traj, x))  # store state history

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            # ob[:, 0] -> The full first row of the array (all X numbers)
            # ob[:, 1] -> The full second row of the array (all Y numbers)
            #plt.plot(ob[:, 0], ob[:, 1], "ok")
            for obx,oby,obs in np.nditer([ob[:, 0], ob[:, 1], ob[:, 2]]):
                patch=plt.Circle((obx, oby), obs, color='black', fill=True)
                tmp=plt.gca()
                tmp.add_patch(patch)
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
