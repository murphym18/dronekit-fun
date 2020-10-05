import math

import numpy as np
from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative

import basic_control as bctl
from ned_frame import NedFrame
from ned_utilities import ned_controller
from ned_plotter import NedData



def magnitude3d(vector):
    x = vector[0]
    y = vector[1]
    z = vector[2]
    return math.sqrt(x*x + y*y + z*z)

def find_tangent_vector(drone_lla, circle_center_lla, is_clockwise=True):
    """Return tangent vector around the circle at current pos
    This is a unit vector in NED coordinates
    """
    # tangent = cross_product(up, p_Drone2Center_N) if clockwise
    # tangent = cross_product(down, p_Drone2Center_N) if counter-clockwise
    origin = NedFrame(drone_lla)
    # p_Drone2Center_N is the vector that points from the drone to the center
    # if I make a coordinate system where the drone is at origin, than the
    # coordinates of the center equal the p_Drone2Center_N
    p_Drone2Center_N = origin.find_ned(circle_center_lla)

    v = np.array([0.0, 0.0, -1.0]) # up vector
    if not is_clockwise:
        v = v * -1.0 
    t = np.cross(v, p_Drone2Center_N)
    t = t * (1.0 / magnitude3d(t))
    return t
    
def progress_angle(start_position_lla, center_position_lla, current_position_lla, is_clockwise=True):
    """calculate the angle that indicates our progress around the circle.
    This angle would be a value from 0 to 360 degrees. Where 0 degrees means no
    progress and 359 degrees would mean we are almost done.
    """
    # First we create two vectors. The first vector points from the center of
    # the circle to the drone's start position called p_Center2Start_N.
    origin = NedFrame(center_position_lla)
    p_Center2Start_N = origin.find_ned(start_position_lla)

    #  And the second vector points from the center of the circle to the drone's
    # current position, call it p_Center2Drone_N.
    p_Center2Drone_N = origin.find_ned(current_position_lla)

    # To get the angle between these vectors we can rearrange this formula that
    # uses the dot product:
    # theta = acos ( (x dot y) / (|x| * |y|) )
    numerator = np.dot(p_Center2Start_N, p_Center2Drone_N)
    denominator = magnitude3d(p_Center2Start_N) * magnitude3d(p_Center2Drone_N)
    theta = math.acos(numerator / denominator)

    # theta is an angle from 0 to pi radians
    # we want an angle from 0 to 2*pi
    # we need to find out if the drone is more than 90 degrees from the first
    # quarter of the circle.
    # We need to find the waypoint that's 25% around the circle
    # this point can be found using the cross_product(down, start) for 
    # clockwise and cross_product(up, start) for counter-clockwise
    # let v = down if clockwise or up if not
    v = [0.0, 0.0, 1.0]
    if not is_clockwise:
        v = v * -1.0
    
    waypoint_25_percent = np.cross(v, p_Center2Start_N)
    # normalize
    waypoint_25_percent = waypoint_25_percent * (1.0 / magnitude3d(waypoint_25_percent))
    # scale to match the radius
    waypoint_25_percent_N = waypoint_25_percent * magnitude3d(p_Center2Start_N)

    # now we find the angle between our current position and the waypoint at 25%
    numerator = np.dot(p_Center2Drone_N, waypoint_25_percent_N)
    denominator = magnitude3d(p_Center2Drone_N) * magnitude3d(waypoint_25_percent_N)
    a = math.acos(numerator / denominator)

    # if a is bigger than pi / 2 then we are more than 50% around the circle
    if a > math.pi / 2.0:
        theta = 2*math.pi - theta
    return math.degrees(theta)

def main():
    drone, sitl = bctl.connect_virtual_vehicle(0, [41.714827, -86.241931, 0])
    bctl.arm_and_takeoff(drone, 10)
    ned = ned_controller()
    home = bctl.get_position(drone)
    data = NedData(home)
    start_pos = home

    center = home.move_ned(0, 10, 0)

    progress = 0
    print("flying first quarter of circle")
    while progress < 90.0:
        pos = bctl.get_position(drone)
        data.log_lla(pos)
        progress = progress_angle(start_pos, center, pos)
        v = find_tangent_vector(pos, center) * 3.0
        ned.send_ned_velocity(v[0], v[1], v[2], drone)

    print("flying the rest of the circle")
    while progress > 45.0:
        pos = bctl.get_position(drone)
        data.log_lla(pos)
        progress = progress_angle(start_pos, center, pos)
        v = find_tangent_vector(pos, center) * 3.0
        ned.send_ned_velocity(v[0], v[1], v[2], drone)
    
    print("Returning to Launch")
    drone.mode = VehicleMode("RTL")
    # Close vehicle object before exiting script
    print("Close vehicle object")
    drone.close()

    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()

    data.plot()


if __name__ == "__main__":
    main()    