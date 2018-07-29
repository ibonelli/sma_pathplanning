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
        self.sensor_radius = 10  # [m]
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
            p.col,p.r = self.lidar_limits(x, r, ob)

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

    def detect_collision_point(self, p1, p2, q, r):
        intersects, values = self.getSegmentCircleIntersection(p1,p2,q,r)
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

    # We get the limit for an specific angle.
    # Almost the same as the obstacle_detection()
    def lidar_limits(self, x, r, ob):
        oi = []
        for obx,oby,obs in np.nditer([ob[:, 0], ob[:, 1], ob[:, 2]]):
            intersects, limit = self.detect_collision_point(x, r, np.array([obx, oby]), obs)
            if (intersects):
                #print "Intersection at " + str(limit)
                oi.append(limit)
        if (len(oi) == 0):
            return intersects, r
        elif (len(oi) == 1):
            return intersects, oi[0]
        else:
            min_val = 99999999
            val_to_return = None
            for p in oi:
                h = np.linalg.norm(p)
                if (h < min_val):
                    min_val = h
                    val_to_return = p
            return intersects, p

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

    # Check if an object is on the robot's path
    # Using scalar product, see:
    #     <Dropbox>/Maestria/TFE/Collision_Detection.txt
    #     <Dropbox>/Maestria/TFE/PathPlanning/P_is_rectangle*.py
    def isPinRectangle(self, r, P):
        C = np.array(r[0][0], r[0][1])
        v = np.array([P[0] - r[0][0], P[1] - r[0][1]])
        v1 = np.array([r[1][0] - r[0][0], r[1][1] - r[0][1]])
        v2 = np.array([r[3][0] - r[0][0], r[3][1] - r[0][1]])

        check1 = np.dot(v,v1)
        v1_lim = np.dot(v1,v1)
        check2 = np.dot(v,v2)
        v2_lim = np.dot(v2,v2)
        ok1 = False
        ok2 = False

        if(0 <= check1 and check1 <= v1_lim):
            ok1 = True
        if(0 <= check2 and check2 <= v2_lim):
            ok2 = True

        if(ok1 and ok2):
            return True
        else:
            return False

    def getPathLimits(self, org, dst, ang):
        r = np.zeros(shape=(4,2))
        r_sin = self.robot_radius * math.sin(ang)
        r_cos = self.robot_radius * math.cos(ang)
        r[0] = [org[0] + r_cos, org[1] - r_sin]
        r[1] = [org[0] - r_cos, org[1] + r_sin]
        dx = dst[0]
        dy = dst[1]
        r[2] = [dx + r_cos, dy - r_sin]
        r[3] = [dx - r_cos, dy + r_sin]
        return r

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
        p.col,p.r = self.lidar_limits(x, goal, ob)

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
