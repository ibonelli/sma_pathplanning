import math
import numpy as np
import random

class rAgent():
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
 
    def obstacle_detection(self, x, r, ang, ob):
        step = 2 * self.robot_radius - self.robot_radius * 0.01
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


    def motion(self, x, ob, ang, vel):
        col = False

        x_dir = math.cos(ang)
        y_dir = math.sin(ang)

        rx = x[0] + x_dir * vel
        ry = x[1] + y_dir * vel

        r = np.array([rx, ry])

        r,col = self.obstacle_detection(x, r, ang, ob)

        return r,col


    def random_control(self):
        ang = random.random() * 2 * math.pi
        vel = random.random() * self.step
        ticks = random.randint(0,10)

        return ang, vel, ticks
