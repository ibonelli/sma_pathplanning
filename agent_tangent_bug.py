import math
import numpy as np
import random
import matplotlib.pyplot as plt

class LidarLimits:
    def __init__(self):
        self.angle = 0
        self.oi = np.zeros(2)
        self.dist = 0

    def print_values(self):
        print "angle: " + str(self.angle)
        print "oi: " + str(self.oi)
        print "dist: " + str(self.dist)


class LidarPoint:
    def __init__(self):
        self.angle = 0
        self.r = np.zeros(2)
        self.dist = 0
        self.col = False

    def print_values(self):
        print "angle: " + str(self.angle)
        print "oi: " + str(self.oi)
        print "dist: " + str(self.dist)
        print "collision: " + str(self.col)


class atan1Agent():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.step = 1  # [m]
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.01  # [m/s]
        self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s]
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.robot_radius = 1.0  # [m]
        self.sensor_radius = 6  # [m]
        self.sensor_angle_steps = 36  # [rad]
        self.lidar_object_limit = 3
        self.imagefile = 1

    # We get the limit for each LIDAR point
    # We will have as many limits as self.sensor_angle_steps
    # We store x, y and d (size of the vector)
    def lidar(self, x, ob):
        limit = []
        angle_step = 2 * math.pi / self.sensor_angle_steps

        for i in xrange(self.sensor_angle_steps):
            p = LidarPoint()
            p.ang = angle_step * i
            rx = x[0] + self.sensor_radius * math.cos(angle_step * i)
            ry = x[1] + self.sensor_radius * math.sin(angle_step * i)
            r = np.array([rx, ry])
            p.r,p.dist,p.col = self.lidar_limits(x, r, p.ang, ob)

            limit.append(p)

        return limit

    # We get the limit for an specific angle.
    # Almost the same as the obstacle_detection()
    def lidar_limits(self, x, r, ang, ob):
        # step will be the 99% of the diameter of the robot
        step = 2 * self.robot_radius * 0.99

        dx = x[0] - r[0]
        dy = x[1] - r[1]
        distance = math.sqrt(math.pow(dx,2)+math.pow(dy,2))

        steps_to_do = int(math.ceil(distance/step))
        step = distance / steps_to_do
        stepx = math.cos(ang) * step
        stepy = math.sin(ang) * step

        collision = False
        smallest = 9999999

        for i in xrange(steps_to_do):
            for obx,oby,obs in np.nditer([ob[:, 0], ob[:, 1], ob[:, 2]]):
                # TODO - Is obstacle in path?
                # Need to use : https://martin-thoma.com/how-to-check-if-a-point-is-inside-a-rectangle/
                # For rectangle we use: x, ob & r_rect = obs+self.robot_radius
                xloc = x[0] + stepx * i
                yloc = x[1] + stepy * i
                dx = xloc - obx
                dy = yloc - oby
                dist = math.sqrt(math.pow(dx,2)+math.pow(dy,2))
                if (dist < self.lidar_object_limit):
                    if (dist < smallest):
                        collision = True
                        dx = x[0] - obx
                        dy = x[1] - oby
                        smallest = math.sqrt(math.pow(dx,2)+math.pow(dy,2))
                        r = np.array([obx, oby])

        if (not collision):
            smallest = self.sensor_radius

        return r,smallest,collision

    # Obstacle detection for the robot
    # Same algorithm used by random movement agent
    def obstacle_detection(self, x, r, ang, ob):
        step = 2 * self.robot_radius * 0.99
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
                if (dist < (obs+self.robot_radius)):
                    collision = True
                    rx = x[0] + stepx * (i-1)
                    ry = x[1] + stepy * (i-1)
                    r = np.array([rx, ry])
                    print "Collision: obx [" + str(obx) + "," + str(oby) + "] | ang: " + str(ang)
                    return r,collision

        return r,collision

    def motion(self, x, ob, ang, vel):
        x_dir = math.cos(ang)
        y_dir = math.sin(ang)
        rx = x[0] + x_dir * vel
        ry = x[1] + y_dir * vel
        r = np.array([rx, ry])

        r,col = self.obstacle_detection(x, r, ang, ob)

        return r,col

    # Initially followed description on http://www.cs.bilkent.edu.tr/~culha/cs548/hw1/
    #        @ "2.1. Motion to Goal Behaviour" we get o1 & o2
    # But that's a simplistic approach, we have many oi
    # I'll use the simplest differential/borders algorithm to identify the oi list
    def get_limits(self, x, goal, limit):
        oi_list = []
        di_list = []

        angle_step = 2 * math.pi / self.sensor_angle_steps
        last = self.sensor_angle_steps - 1

        follow = False
        add_to_list = False
        for i in range(self.sensor_angle_steps):
            if (limit[i].dist == self.sensor_radius and follow == False):
                follow = True
                add_to_list = True
                l = LidarLimits()
                l.oi[0] = limit[i].r[0]
                l.oi[1] = limit[i].r[1]
            elif (limit[i].dist != self.sensor_radius and follow == True):
                follow = False
                add_to_list = True
                l = LidarLimits()
                l.oi[0] = limit[i-1].r[0]
                l.oi[1] = limit[i-1].r[1]
            # Only one place to add to list
            if (add_to_list == True):
                add_to_list = False
                l.angle = angle_step * i
                path1 = math.sqrt((x[0] - l.oi[0])**2 + (x[1] - l.oi[1])**2)
                path2 = math.sqrt((l.oi[0] - goal[0])**2 + (l.oi[1] - goal[1])**2)
                l.dist = path1 + path2
                oi_list.append(l)

        self.graph_limits(limit, di_list, oi_list)
        return oi_list

    def print_oi_list(self, oi_list):
        i = 1

        for oi in oi_list:
            print "o" + str(i) + " values ------------"
            oi.print_values()
            i += 1

        print "----------------------"

    def get_min_oi(self, oi_list):
        selected = oi_list[0]

        for oi in oi_list:
            if (oi.dist < selected.dist):
                selected = oi

        return selected

    def tangentbug_control(self, x, ob, goal, limit):
        # Modeled using description at:
        #         http://www.cs.bilkent.edu.tr/~culha/cs548/hw1/
        # And with some graphical aid from:
        #         https://www.cs.cmu.edu/~motionplanning/student_gallery/2006/st/hw2pub.htm
        oi_list = self.get_limits(x, goal, limit)
        self.print_oi_list(oi_list)

        p = LidarPoint()
        ang = math.atan2(goal[1]-x[1],goal[0]-x[0])
        p.r,p.dist,p.col = self.lidar_limits(x, goal, ang, ob)

        if (not p.col):
            # No collission in sight, we can move directly to target
            ang = math.atan2(goal[1]-x[1],goal[0]-x[0])
            print "Moving directly to target ---------"
            print "Angle: " + str(ang)
        else:
            # Collission in sight, need to decide best strategy
            if (len(oi_list) >= 1):
                oi_dfollowed = self.get_min_oi(oi_list)
                print "Oi to follow --------------"
                oi_dfollowed.print_values()
                ang = oi_dfollowed.angle
            else:
                print "Can not find best path... Random move!"
                ang = random.random() * 2 * math.pi

        vel = self.step
        ticks = 1

        return ang, vel, ticks

    def graph_lidar(self, x, goal, limit, ob, frame):
        # Graph LIDAR limit
        plt.cla()
        plt.plot(x[0], x[1], "xr")
        plt.plot(goal[0], goal[1], "xb")
        fil,col = np.shape(limit)
        plt.plot(limit[:,0], limit[:,1], "b.")
        plt.plot(ob[:, 0], ob[:, 1], "ok")
        plt.axis("equal")
        plt.grid(True)
        filenumber = format(frame, '05d')
        plt.savefig("/home/ignacio/Downloads/PyPlot/tangentbug_" + filenumber + ".png")

    def graph_limits(self, limit, di_limit, oi_list):
        filenumber = format(self.imagefile, '05d')
        self.imagefile += 1

        # Graph LIDAR limit
        plt.cla()
        for l in limit:
            plt.plot(l.r[0], l.r[1], "b.")
        if (len(oi_list) >= 1):
            for l in oi_list:
                plt.plot(l.oi[0], l.oi[1], "xr")
        plt.axis("equal")
        plt.grid(True)
        plt.savefig("/home/ignacio/Downloads/PyPlot/limit_" + filenumber + ".png")

    def graph_di_list(limit_list,file_name,file_number):
        plt.cla()
        i=0
        for l in limit_list:
            plt.plot(i, l, 'b-')
            i+=1
        plt.grid(True)
        file_number_to_use = format(file_number, '05d')
        plt.savefig("/home/ignacio/Downloads/PyPlot/limit_" + file_name + "_" + file_number_to_use + ".png")

    def graph_limit_list(limit_list,file_name,file_number):
        plt.cla()
        i=0
        for l in limit_list:
            plt.plot(i, l.dist, 'b-')
            i+=1
        plt.grid(True)
        file_number_to_use = format(file_number, '05d')
        plt.savefig("/home/ignacio/Downloads/PyPlot/limit_" + file_name + "_" + file_number_to_use + ".png")

    def print_limit_list(limit_list):
        for l in limit_list:
            print l.dist,
        print ""
