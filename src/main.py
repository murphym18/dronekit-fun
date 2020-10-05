from mathtools import Lla, Pvector, Nvector
from basic_control import connect_virtual_vehicle, arm_and_takeoff
from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative
from ned_utilities import ned_controller
from flight_plotter import Location, CoordinateLogger, GraphPlotter
from ned_plotter import NedData
from ned_frame import find_ned
import numpy as np
import math


MAX_SPEED = 5.0


def get_position(vehicle):
    return Lla(vehicle.location.global_relative_frame.lat,
               vehicle.location.global_relative_frame.lon,
               vehicle.location.global_relative_frame.alt)


def is_before_diverting(start_location, current_location, end_location):
    """returns true when the current location is less than 50% of the way to end location"""
    a = Lla(start_location.latitude, start_location.longitude,
            start_location.altitude)
    b = Lla(end_location.latitude, end_location.longitude,
            end_location.altitude)
    current = Lla(current_location.latitude, current_location.longitude,
                  current_location.altitude)
    d_full = a.distance(b)
    d_remaining = current.distance(b)
    return d_remaining / d_full > 0.5

def direction_not_changed(vehicle, new_direction):
    v = vehicle.velocity
    denominator = np.linalg.norm(v) * np.linalg.norm(new_direction)
    if denominator != 0:
        theta = math.acos(np.dot(new_direction, v) / denominator)
        return np.degrees(theta) > 2.5
    else:
        return True

def find_velocity(a, b, speed):
    n, e, d = find_ned(a, b)
    velocity = np.array([n, e, d])
    velocity = velocity  * (1.0 / np.linalg.norm(velocity))
    velocity = velocity * speed
    return velocity

drone, sitl = connect_virtual_vehicle(0, [41.714827, -86.241931, 0])
arm_and_takeoff(drone, 10)
home = get_position(drone)
A = home
B = A.move_ned(40, 0, 0)

controller = ned_controller()
data = NedData(home)
data.log_poi(A, "A")
data.log_poi(B, "B")


while is_before_diverting(A, get_position(drone), B):
    print("flying toward B")
    velocity = find_velocity(A, B, MAX_SPEED)
    print(velocity)
    controller.send_ned_velocity(velocity[0], velocity[1], velocity[2], drone)
    data.log_lla(get_position(drone))

data.log_poi(get_position(drone), "Divert")
# divert to the left
# divert_direction = np.cross([0.0, 0.0, -1.0], drone.velocity)
divert_direction = np.array(drone.velocity) * -1
divert_direction = divert_direction * 10.0 / np.linalg.norm(divert_direction)
while direction_not_changed(drone, divert_direction):
    print("diverting!")
    velocity = divert_direction
    controller.send_ned_velocity(velocity[0], velocity[1], velocity[2], drone)
    data.log_lla(get_position(drone))


print("Returning to Launch")
drone.mode = VehicleMode("RTL")
# Close vehicle object before exiting script
print("Close vehicle object")
drone.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

data.plot()



