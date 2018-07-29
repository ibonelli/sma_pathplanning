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
from agent_random import rAgent

# Globals
show_animation = True

def main():
    print(__file__ + " start!!")
    
    # Random1 --------
    # initial state [x, y]
    r1_x = np.array([40, 20])
    # Saving trajectory
    r1_traj = np.array(r1_x)
    r1_ticks = 0

    # DWA ------------
    # initial state [x, y, yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([10, 10, math.pi / 8.0, 0.0, 0.0])
    # goal position [x, y]
    goal = np.array([50, 50])
    # Saving trajectory
    traj = np.array(x)
    u = np.array([0.0, 0.0])

    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt("world03.csv")

    dwa = dwaAgent()
    r1 = rAgent()

    for i in range(1000):
        # Random1 - Calculating trajectory
        if (r1_ticks == 0):
            r1_ang, r1_vel, r1_ticks = r1.random_control()
        else:
            r1_ticks-=1
        # Random1 - Calculating movement
        dwa_ob = np.array([x[0], x[1], dwa.robot_radius])
        ob4r1 = np.vstack((ob,dwa_ob))
        r1_x, r1_col = r1.motion(r1_x, ob4r1, r1_ang, r1_vel)
        if (r1_col):
            r1_ticks = 0
        r1_traj = np.vstack((r1_traj, r1_x))  # store state history

        # DWA
        r1_ob = np.array([r1_x[0], r1_x[1], r1.robot_radius])
        ob4dwa = np.vstack((ob,r1_ob))
        u, ltraj = dwa.dwa_control(x, u, goal, ob4dwa)
        x = dwa.motion(x, u)
        print "Pos " + str(i) + " - x: " + str(x)
        traj = np.vstack((traj, x))  # store state history

        if show_animation:
            plt.cla()
            # Random1
            plt.plot(r1_x[0], r1_x[1], "xr")
            # DWA
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xg")
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
        # Random1
        plt.plot(r1_traj[:, 0], r1_traj[:, 1], "-r")
        # DWA
        plt.plot(traj[:, 0], traj[:, 1], "-g")
        plt.show()


if __name__ == '__main__':
    main()
