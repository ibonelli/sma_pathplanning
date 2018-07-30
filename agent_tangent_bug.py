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
            p.ang = angle_step * i
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
            return False, r
        elif (len(oi) == 1):
            return True, oi[0]
        else:
            min_val = 99999999 # Try float("inf")
            val_to_return = None
            for p in oi:
                h = np.linalg.norm(p)
                if (h < min_val):
                    min_val = h
                    val_to_return = p
            return True, p

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

    # From the LIDAR list we get best possible paths
    def get_limits(self, x, goal, limit, ob):
        oi_list = []

        angle_step = 2 * math.pi / self.sensor_angle_steps
        last = self.sensor_angle_steps - 1
        path_clear = False

        for i in range(self.sensor_angle_steps):
            if (limit[i].dist >= self.lidar_object_limit and path_clear == False):
                cant = 1
                ang_ini = angle_step * i
                path_clear = True
            if (limit[i].dist >= self.lidar_object_limit and path_clear == True):
                cant += 1
                path_clear = True
            if (limit[i].dist <= self.lidar_object_limit and path_clear == True):
                ang_fin = angle_step * i
                path_clear = False
                # Less than 30 degree aperture:
                #   => We stablish as trajectory center point
                if ((ang_fin-ang_ini) < (math.pi/6)):
                    cent_ang = (ang_ini+ang_fin)/2
                    print "Ang < 30 : " + str(cent_ang)
                    l = self.save_limit(x, goal, cent_ang, ob)
                    oi_list.append(l)
                # More than 120 degree aperture:
                #   => We stablish as trajectory tangent points
                elif ((ang_fin-ang_ini) > (4*math.pi/6)):
                    cent_ang = (ang_ini+ang_fin)/2
                    print "Ang > 120 : " + str(cent_ang)
                    p_ang1 = cent_ang + math.pi/2
                    l = self.save_limit(x, goal, p_ang1, ob)
                    oi_list.append(l)
                    p_ang2 = cent_ang - math.pi/2
                    l = self.save_limit(x, goal, p_ang2, ob)
                    oi_list.append(l)
                # In between:
                #   => We use ang_ini & ang_fin as points
                else:
                    print "Ang in between. ang_ini: " + str(ang_ini) + " & ang_fin: " + str(ang_fin)
                    l = self.save_limit(x, goal, ang_ini, ob)
                    oi_list.append(l)
                    l = self.save_limit(x, goal, ang_fin, ob)
                    oi_list.append(l)

        self.graph_limits(limit, oi_list)
        return oi_list

    # We save a limit from it's angle
    def save_limit(self, x, goal, ang, ob):
        dst_x = math.cos(ang) * self.sensor_radius + x[0]
        dst_y = math.sin(ang) * self.sensor_radius + x[1]
        intersects, p = self.lidar_limits(x, np.array([dst_x, dst_y]), ob)
        l = LidarPoint()
        l.col = intersects
        l.angle = ang
        l.r[0] = p[0]
        l.r[1] = p[1]
        path1 = math.sqrt((x[0] - l.r[0])**2 + (x[1] - l.r[1])**2)
        path2 = math.sqrt((l.r[0] - goal[0])**2 + (l.r[1] - goal[1])**2)
        l.dist = path1 + path2
        return l

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
            if (oi.dist < selected.dist and oi.col == False):
                selected = oi

        return selected

    def tangentbug_control(self, x, ob, goal, limit):
        # Modeled using description at:
        #         http://www.cs.bilkent.edu.tr/~culha/cs548/hw1/
        # And with some graphical aid from:
        #         https://www.cs.cmu.edu/~motionplanning/student_gallery/2006/st/hw2pub.htm
        oi_list = self.get_limits(x, goal, limit, ob)
        self.print_oi_list(oi_list)

        if (len(oi_list) == 0):
            # No collission in sight, we can move directly to target
            ang = math.atan2(goal[1]-x[1],goal[0]-x[0])
            print "Moving directly to target ---------"
            print "Angle: " + str(ang)
        elif (len(oi_list) >= 1):
            # Collission in sight, need to decide best strategy
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
