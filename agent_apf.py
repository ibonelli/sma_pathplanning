import math
import numpy as np
import matplotlib.pyplot as plt

class apfAgent():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.KP = 5.0  # attractive potential gain
        self.ETA = 100.0  # repulsive potential gain
        self.AREA_WIDTH = 30.0  # potential area width [m]
        self.grid_size = 0.5  # potential grid size [m]
        self.robot_radius = 5.0  # robot radius [m]
        self.show_animation = True

    def calc_potential_field(self, gx, gy, ox, oy):
        minx = min(ox) - self.AREA_WIDTH / 2.0
        miny = min(oy) - self.AREA_WIDTH / 2.0
        maxx = max(ox) + self.AREA_WIDTH / 2.0
        maxy = max(oy) + self.AREA_WIDTH / 2.0
        xw = int(round((maxx - minx) / self.grid_size))
        yw = int(round((maxy - miny) / self.grid_size))

        # calc each potential
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            x = ix * self.grid_size + minx

            for iy in range(yw):
                y = iy * self.grid_size + miny
                ug = self.calc_attractive_potential(x, y, gx, gy)
                uo = self.calc_repulsive_potential(x, y, ox, oy)
                uf = ug + uo
                pmap[ix][iy] = uf

        return pmap, minx, miny

    def calc_attractive_potential(self, x, y, gx, gy):
        return 0.5 * self.KP * np.hypot(x - gx, y - gy)

    def calc_repulsive_potential(self, x, y, ox, oy):
        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for i in range(len(ox)):
            d = np.hypot(x - ox[i], y - oy[i])
            if dmin >= d:
                dmin = d
                minid = i

        # calc repulsive potential
        dq = np.hypot(x - ox[minid], y - oy[minid])

        if dq <= self.robot_radius:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * self.ETA * (1.0 / dq - 1.0 / self.robot_radius) ** 2
        else:
            return 0.0

    def get_motion_model(self):
        # dx, dy
        motion = [[1, 0],
                  [0, 1],
                  [-1, 0],
                  [0, -1],
                  [-1, -1],
                  [-1, 1],
                  [1, -1],
                  [1, 1]]

        return motion

    def draw_heatmap(self, data):
        data = np.array(data).T
        plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

    def potential_field_planning(self, sx, sy, gx, gy, ox, oy):
        # calc potential field
        pmap, minx, miny = self.calc_potential_field(gx, gy, ox, oy)

        # search path
        d = np.hypot(sx - gx, sy - gy)
        ix = round((sx - minx) / self.grid_size)
        iy = round((sy - miny) / self.grid_size)
        gix = round((gx - minx) / self.grid_size)
        giy = round((gy - miny) / self.grid_size)

        if self.show_animation:
            self.draw_heatmap(pmap)
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")

        return d, ix, iy, gix, giy

    def potential_field_planning_navigation(self, sx, sy, gx, gy, ox, oy, d, ix, iy, gix, giy):
        rx, ry = [sx], [sy]
        motion = self.get_motion_model()
        while d >= self.grid_size:
            minp = float("inf")
            minix, miniy = -1, -1
            for i in range(len(motion)):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(pmap) or iny >= len(pmap[0]):
                    p = float("inf")  # outside area
                else:
                    p = pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix = minix
            iy = miniy
            xp = ix * self.grid_size + minx
            yp = iy * self.grid_size + miny
            d = np.hypot(gx - xp, gy - yp)
            rx.append(xp)
            ry.append(yp)

            if self.show_animation:
                plt.plot(ix, iy, ".r")
                plt.pause(0.01)

        print("Goal!!")

        return rx, ry
