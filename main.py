"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
	# https://docs.scipy.org/doc/numpy/user/basics.creation.html
import matplotlib.pyplot as plt
	# https://matplotlib.org/users/pyplot_tutorial.html
import random

# Modules
from robot import Config

# Globals
show_animation = True

def obstacle_detection(x, r, ang, ob, config):
    step = 2 * config.robot_radius - config.robot_radius * 0.01
    collision = False

    dx = x[0] - r[0]
    dy = x[1] - r[1]
    distance = math.sqrt(math.pow(dx,2)+math.pow(dy,2))

    steps_to_do = int(math.ceil(distance/step))
    step = distance / steps_to_do
    stepx = math.cos(ang) * step
    stepy = math.sin(ang) * step

    for i in xrange(steps_to_do):
        for obx,oby,obs in np.nditer([ob[:, 0], ob[:, 1], ob[:, 2]]):
            # print '(' + str(obx) + ',' + str(oby) + ',' + str(obs) + ')'
            xloc = x[0] + stepx * i
            yloc = x[1] + stepy * i
            dx = xloc - obx
            dy = yloc - oby
            dist = math.sqrt(math.pow(dx,2)+math.pow(dy,2))
            if (dist < (obs+config.robot_radius)):
                collision = True
                rx = x[0] + stepx * (i-1)
                ry = x[1] + stepy * (i-1)
                r = np.array([rx, ry])
                print r
                return r,collision

    # 2D Collision Detection (Colision circular)
    #   https://developer.mozilla.org/es/docs/Games/Techniques/2D_collision_detection

    #--------------
    #var circle1 = {radius: 20, x: 5, y: 5};
    #var circle2 = {radius: 12, x: 10, y: 5};
    #
    #var dx = circle1.x - circle2.x;
    #var dy = circle1.y - circle2.y;
    #var distance = Math.sqrt(dx * dx + dy * dy);
    #
    #if (distance < circle1.radius + circle2.radius) {
    #    // collision detected!
    #}
    #--------------

    return r,collision


def motion(x, ob, ang, vel, config):
    col = False

    x_dir = math.cos(ang)
    y_dir = math.sin(ang)

    rx = x[0] + x_dir * vel
    ry = x[1] + y_dir * vel

    r = np.array([rx, ry])

    r,col = obstacle_detection(x, r, ang, ob, config)

    return r,col


def random_control(config):
    ang = random.random() * 2 * math.pi
    vel = random.random() * config.step
    ticks = random.randint(0,10)

    return ang, vel, ticks


def main():
    print(__file__ + " start!!")
    # initial state [x, y]
    x = np.array([10, 10])
    # goal position [x, y]
    goal = np.array([50, 50])
    # obstacles [ob1(x,y,r), ob2(x,y,r), ....]
    # x,y coord and obstacle radius
    ob = np.loadtxt("myworld.csv")

    config = Config()

    print "main()"
    print "config: " + str(config)

    traj = np.array(x)
    ticks = 0

    for i in range(1000):
        if (ticks == 0):
            ang, vel, ticks = random_control(config)
        else:
            ticks-=1

        x,col = motion(x, ob, ang, vel, config)
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
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.show()


if __name__ == '__main__':
    main()
