import math
import numpy as np
import random
import matplotlib.pyplot as plt

class LidarPoint:
    def __init__(self):
        self.angle = 0
        self.r = np.zeros(2)
        self.dist = 0
        self.col = False

    def print_values(self):
        print "angle: " + str(self.angle)
        print "oi: " + str(self.r)
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
        self.sensor_radius = 10  # [m]
        self.sensor_angle_steps = 36  # [rad]
        self.lidar_object_limit = 4
        self.imagefile = 1

    # We get the limit for each LIDAR point
    # We will have as many limits as self.sensor_angle_steps
    # We store x, y and d (size of the vector)
    def lidar(self, x, ob):
        limit = []
        angle_step = 2 * math.pi / self.sensor_angle_steps

        for i in xrange(self.sensor_angle_steps):
            p = LidarPoint()
            p.angle = angle_step * i
            rx = x[0] + self.sensor_radius * math.cos(angle_step * i)
            ry = x[1] + self.sensor_radius * math.sin(angle_step * i)
            r = np.array([rx, ry])
            p.col,p.r = self.lidar_limits(x, r, ob)
            p.dist = math.sqrt(math.pow(p.r[0]-x[0],2)+math.pow(p.r[1]-x[1],2))

            limit.append(p)

        return limit

    def getSegmentCircleIntersection(self, p1, p2, q, r):
        v = np.array([p2[0]-p1[0], p2[1]-p1[1]])
        a = v.dot(v)
        b = 2 * v.dot(np.array([p1[0]-q[0], p1[1]-q[1]]))
        c = p1.dot(p1) + q.dot(q) - 2 * p1.dot(q) - r**2
        disc = b**2 - 4 * a * c
        if disc < 0:
            return False, None
        sqrt_disc = math.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)

        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return False, None
        else:
            i = []
            if (0 <= t1 <= 1):
                i1 = v.dot(t1) + p1
                i.append(i1)
            if (0 <= t2 <= 1):
                i2 = v.dot(t2) + p1
                i.append(i2)
            return True, i

    # We get the limit for an specific angle.
    # Where:
    #       p1: Is the first point of a segment
    #       p2: Is the second point of a sagment
    #        q: Is the center of a circle that could intersect with the segment
    #        r: Is the radius of the circle that could intersect the segment
    def detect_collision_point(self, p1, p2, q, r):
        intersects, values = self.getSegmentCircleIntersection(p1,p2,q,r)
        # This code returns the proper intersection point
        if (intersects):
            if (len(values) == 1):
                return intersects, values[0]
            else:
                m1 = np.linalg.norm(values[0])
                m2 = np.linalg.norm(values[1])
                if(m1>m2):
                    return intersects, values[1]
                else:
                    return intersects, values[0]
        else:
            return intersects, None

    # We get the object closest to the LIDAR looking angle.
    # Where:
    #       p1: Is the first point of a segment
    #       p2: Is the second point of a sagment
    #        q: Is the center of a circle that could intersect with the segment
    #        r: Is the radius of the circle that could intersect the segment
    def detect_collision_object(self, p1, p2, q, r):
        intersects, values = self.getSegmentCircleIntersection(p1,p2,q,r)
        if (intersects):
            return intersects, q
        else:
            return intersects, None

    # We get the limit for an specific angle.
    # Where:
    #        x: Is the position where LIDAR is measuring
    #        r: Is the current limit LIDAR position being explored for object collision
    #       ob: Is the x,y position of each object along with its radius (obs)
    def lidar_limits(self, x, r, ob):
        oi = []
        for obx,oby,obs in np.nditer([ob[:, 0], ob[:, 1], ob[:, 2]]):
            intersects, limit = self.detect_collision_object(x, r, np.array([obx, oby]), obs)
            if (intersects):
                #print "Intersection at " + str(limit)
                oi.append(limit)
        if (len(oi) == 0):
            return False, r
        elif (len(oi) == 1):
            return True, oi[0]
        else:
            min_val = float("inf")
            val_to_return = None
            for p in oi:
                h = np.linalg.norm(p)
                if (h < min_val):
                    min_val = h
                    val_to_return = p
            return True, p

    def motion(self, x, ob, ang, vel):
        x_dir = math.cos(ang)
        y_dir = math.sin(ang)
        rx = x[0] + x_dir * vel
        ry = x[1] + y_dir * vel
        r = np.array([rx, ry])

        col,r = self.lidar_limits(x, r, ob)

        return r,col

    # From the LIDAR list we get best possible paths
    def get_limits(self, x, goal, limit, ob):
        oi_list = []

        for l in limit:
            if (l.col == False):
                oi_list.append(l)

        self.graph_limits(limit, oi_list)
        return oi_list

    def print_oi_list(self, oi_list):
        i = 1

        for oi in oi_list:
            print "o" + str(i) + " values ------------"
            oi.print_values()
            i += 1

        print "----------------------"

    def get_min_oi(self, oi_list, x, goal):
        selected_dfollowed = self.get_dfollowed(x, oi_list[0].r, goal)
        selected = oi_list[0]

        for oi in oi_list:
            dfollowed = self.get_dfollowed(x, oi.r, goal)
            if (selected_dfollowed > dfollowed):
                selected = oi
                selected_dfollowed = dfollowed

        return selected

    def get_dfollowed(self, x, r, goal):
        dx = r[0] - x[0]
        dy = r[1] - x[1]
        dist_xr = math.sqrt(math.pow(dx,2)+math.pow(dy,2))
        dx = goal[0] - r[0]
        dy = goal[1] - r[1]
        dist_rgoal = math.sqrt(math.pow(dx,2)+math.pow(dy,2))
        oi_followed = dist_xr + dist_rgoal

        return oi_followed

    def tangentbug_control(self, x, ob, goal, limit):
        # Modeled using description at:
        #         http://www.cs.bilkent.edu.tr/~culha/cs548/hw1/
        # And with some graphical aid from:
        #         https://www.cs.cmu.edu/~motionplanning/student_gallery/2006/st/hw2pub.htm
        oi_list = self.get_limits(x, goal, limit, ob)
        #self.print_oi_list(oi_list)

        if (len(oi_list) == 0):
            print "Can not find best path... Random move!"
            ang = random.random() * 2 * math.pi
        else:
            # We need to get best path
            oi_dfollowed = self.get_min_oi(oi_list, x, goal)
            print "Oi to follow --------------"
            oi_dfollowed.print_values()
            ang = oi_dfollowed.angle

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

    def graph_limits(self, limit, oi_list):
        filenumber = format(self.imagefile, '05d')
        self.imagefile += 1

        # Graph LIDAR limit
        plt.cla()
        for l in limit:
            plt.plot(l.r[0], l.r[1], "b.")
        if (len(oi_list) >= 1):
            for l in oi_list:
                plt.plot(l.r[0], l.r[1], "xr")
        plt.axis("equal")
        plt.grid(True)
        plt.savefig("/home/ignacio/Downloads/PyPlot/limit_" + filenumber + ".png")

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
