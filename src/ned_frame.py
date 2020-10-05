from mathtools import Lla, Nvector, Pvector
import numpy as np

def find_ned(origin_position, other_position):
        """Find NED coordinates, given an origin and a position"""
        p_EO_E = origin_position.to_pvector()
        p_EA_E = other_position.to_pvector()

        # Get the vector that points from self.origin to the other_position
        # This vector is in Earth's frame
        # About the naming convention:
        #   the p means its a p-vector as opposed
        #   the OA means it points from "O" (Origin) to A (Point A)
        #   the E means it's in Earth's frame
        p_OA_E = p_EA_E - p_EO_E

        # Find the rotation matrix that converts Earth vectors to NED vectors
        #
        # About the naming convention:
        #   R means its a rotation matrix
        #   EN means it converts vectors in N (NED) coordinates to E (Earth) coordinates
        #       the order E before N comes from matrix vector multiplication.
        #       to do the operation on paper, the NED vector would go on the right of the matrix 
        R_EN = origin_position.n_E2R_EN()
        R_NE = R_EN.T

        # convert the vector from Earth's coordinates to NED coordinates
        p_OA_N = np.dot(R_NE, p_OA_E)
        n, e, d = p_OA_N.ravel()
        return n, e, d


def find_lla(origin_position, north, east, down):
    return origin_position.move_ned(north, east, down).to_lla()

class NedFrame:
    """Build a NED coordinate system at an origin specified by an LLA value
    This class includes two methods to convert LLA values to and from ned coordinates
    """
    def __init__(self, origin):
        self.origin = origin.to_lla()

    def find_ned(self, other_position):
        """Find NED coordinates, given an LLA position"""
        return find_ned(self.origin, other_position)
    
    def find_lla(self, north, east, down):
        """Find the latitude longitude and altitude of the given north, east
        and down coordinates"""
        return self.origin.move_ned(north, east, down).to_lla()
