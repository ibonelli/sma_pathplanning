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
from agent_apf import apfAgent

# Globals
show_animation = True

def main():
    print(__file__ + " start!!")
    # initial state [x, y]
    x = np.array([5, 5])
    # goal position [x, y]
    goal = np.array([50, 50])
    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt("world01.csv")

    traj = np.array(x)
    ticks = 0

    apf = apfAgent()

    #for i in range(50):
    d, ix, iy, gix, giy = apf.potential_field_planning(x[0], x[1], goal[0], goal[1], ob[:, 0], ob[:, 1])
        #ang, vel, ticks = atan1.tangentbug_control(x, ob, goal, limit)
        #x,col = atan1.motion(x, ob, ang, vel)
        #print "======================================================================="
        #print "Step " + str(i) + " | pos: " + str(x) + " | ang: " + str(ang) + " | col: " + str(col)
        #traj = np.vstack((traj, x))  # store state history
        #
        #if show_animation:
        #    plt.cla()
        #    plt.plot(x[0], x[1], "xr")
        #    plt.plot(goal[0], goal[1], "xb")
        #    # ob[:, 0] -> The full first row of the array (all X numbers)
        #    # ob[:, 1] -> The full second row of the array (all Y numbers)
        #    #plt.plot(ob[:, 0], ob[:, 1], "ok")
        #    for obx,oby,obs in np.nditer([ob[:, 0], ob[:, 1], ob[:, 2]]):
        #        patch=plt.Circle((obx, oby), obs, color='black', fill=True)
        #        tmp=plt.gca()
        #        tmp.add_patch(patch)
        #    plt.axis("equal")
        #    plt.grid(True)
        #    plt.pause(0.0001)
        #
        ## check goal
        #if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= atan1.robot_radius:
        #    print("Goal!!")
        #    break

    print("Done")
    if show_animation:
        #plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.show()


if __name__ == '__main__':
    main()
