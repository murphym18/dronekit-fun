import numpy as np

from mathtools import Lla
import matplotlib.pyplot as plt

def find_ned(lla_origin, lla_other):
    p_EO_E = lla_origin.to_pvector()
    p_EA_E = lla_other.to_pvector()
    p_OA_E = p_EA_E - p_EO_E

    R_EN = lla_origin.n_E2R_EN()
    R_NE = R_EN.T

    p_OA_N = np.dot(R_NE, p_OA_E)
    n, e, d = p_OA_N.ravel()
    return n, e, d

def find_lla(lla_origin, north, east, down):
    lla_result = lla_origin.move_ned(north, east, down)
    return lla_result.to_lla()

class NedData:
    def __init__(self, origin_lla):
        self.origin = origin_lla
        self.p_EO_E = self.origin.to_pvector()
        self.n = []
        self.e = []
        self.d = []
        self.point_of_interest_n = []
        self.point_of_interest_e = []
        self.point_of_interest_labels = []

    def log_poi(self, positon_lla, label):
        """label a point of interest the map"""
        n, e, d = find_ned(self.origin, positon_lla)
        self.point_of_interest_n.append(n)
        self.point_of_interest_e.append(e)
        self.point_of_interest_labels.append(label)




    def log_lla(self, pos):
        n, e, d = find_ned(self.origin, pos)
        self.log_ned(n, e, d)
    
    def log_ned(self, n, e, d):
        self.n.append(n)
        self.e.append(e)
        self.d.append(d)
    
    def plot(self):
        fig = plt.figure()
        poi_ax = fig.add_subplot(111)
        # todo make axes square, show major grids
        plt.plot(self.e ,self.n, linewidth=10,color='gray')
        plt.xlabel("East")
        plt.ylabel("North")
        plt.axis('square')        
        plt.grid(True)

        # show the points of interest
        for label, x, y in zip(self.point_of_interest_labels, self.point_of_interest_e, self.point_of_interest_n):
            poi_ax.annotate(label, xy=(x, y), textcoords='data')

        plt.show()

