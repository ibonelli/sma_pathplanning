"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
	# https://docs.scipy.org/doc/numpy/user/basics.creation.html
import matplotlib.pyplot as plt
	# https://matplotlib.org/users/pyplot_tutorial.html
import subprocess

# Modules
from agent_dwa import dwaAgent
from agent_tangent_bug import atan1Agent

# Globals
show_animation = True
file_number = 1
goal_agent = "DWA"
#goal_agent = "tangentBug"
file_path = "/home/ignacio/Maestria/sma_pathplanning/output/"

def main():
    global file_number
    print(__file__ + " start!!")

    # initial state [x, y, yaw(rad), v(m/s), omega(rad/s)]
    if (goal_agent == "DWA"):
        x = np.array([5, 5, math.pi / 8.0, 0.0, 0.0])
    if (goal_agent == "tangentBug"):
        x = np.array([5, 5])

    # goal position [x, y]
    goal = np.array([50, 50])
    # Saving trajectory
    traj = np.array(x)
    u = np.array([0.0, 0.0])

    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt("world05.csv")

    if (goal_agent == "DWA"):
        dwa = dwaAgent()
    if (goal_agent == "tangentBug"):
        atan1 = atan1Agent()

    for i in range(1000):
        # Goal agent
        if (goal_agent == "DWA"):
            u, ltraj = dwa.dwa_control(x, u, goal, ob)
            x = dwa.motion(x, u)
            print "Pos " + str(i) + " - x: " + str(x)
        if (goal_agent == "tangentBug"):
            limit = atan1.lidar(x, ob)
            #graph_lidar(x, goal, limit, ob, i)
            ang, vel, ticks = atan1.tangentbug_control(x, ob, goal, limit)
            x,col = atan1.motion(x, ob, ang, vel)
            print "======================================================================="
            print "Step " + str(i) + " | pos: " + str(x) + " | ang: " + str(ang) + " | col: " + str(col)

        traj = np.vstack((traj, x))  # store state history

        if show_animation:
            plt.cla()
            # DWA & TangentBug
            if (goal_agent == "DWA"):
                plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
                dwa.plot_arrow(x[0], x[1], x[2])
            plt.plot(x[0], x[1], "xg")
            plt.plot(goal[0], goal[1], "ob")
            # ob[:, 0] -> The full first row of the array (all X numbers)
            # ob[:, 1] -> The full second row of the array (all Y numbers)
            #plt.plot(ob[:, 0], ob[:, 1], "ok")
            for obx,oby,obs in np.nditer([ob[:, 0], ob[:, 1], ob[:, 2]]):
                patch=plt.Circle((obx, oby), obs, color='black', fill=True)
                tmp=plt.gca()
                tmp.add_patch(patch)
            plt.axis("equal")
            plt.grid(True)
            # We save to a file
            imagefile = format(file_number, '05d')
            file_number += 1
            plt.savefig(file_path + "/anim_" + imagefile + ".png")
            # And show as we used to
            #plt.pause(0.0001)

        # check goal
        if (goal_agent == "DWA"):
            check_radius = dwa.robot_radius
        if (goal_agent == "tangentBug"):
            check_radius = atan1.robot_radius
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= check_radius:
            print("Goal!!")
            break

    # Didn't work
    #cmd = "convert -delay 0.5 " + path + "anim_*.png -loop 0 -monitor " + path + "movie.gif"
    # Works, but not within Python
    cmd = 'ffmpeg -framerate 25 -pattern_type glob -i "anim_*.png" output.mkv'
    #status = subprocess.call(cmd, shell=True)
    print "First run:"
    print "    " + cmd
    print "And then run:"
    print "    " + "rm " + file_path + "anim_*.png"

    print("Done")
    if show_animation:
        # DWA & TangentBug
        plt.plot(traj[:, 0], traj[:, 1], "-g")
        # We save to a file
        plt.savefig(file_path + "/movie_end.png")
        # And show as we used to
        plt.show()

if __name__ == '__main__':
    main()
