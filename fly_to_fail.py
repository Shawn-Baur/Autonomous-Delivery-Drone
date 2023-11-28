from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
from time import sleep
import argparse
from datetime import datetime

date_time = str(datetime.now())
file = "Fligt_to_Fail_Log.txt"

# Destination cordinates
waypoint1 = [38.403390, -121.398838]

# Height and speed parameters for autonomus flight
height = 17
speed = 10

# Function for writing to file
def filewrite(file, text):
    f = open(file, "a")
    f.write("{}\n".format(text))
    f.close()

# Funciton that identifies the cordinates of the drone, used to get the original takeoff location for return
def identify_home():
    longitude = vehicle.location.global_relative_frame.lat
    latatude = vehicle.location.global_relative_frame.lon

    home_cordinates = [longitude, latatude]
    return home_cordinates
    
# Funciton for connecting to the drone
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600

    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

# Funciton to launch
def launch(vehicle, height):
    vehicle.simple_takeoff(height)

    while True:
        print("Alt: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= height * .95:
            break
        else:
            sleep(1)

# Function to go to cordinates
def go_to_destination(vehicle, destination, height, speed):
    # Split longitude and latatude from destination list
    longitude = destination[0]
    latatude = destination[1]
    
    # Changes airspeed and simple_goto allows flight to cordinates
    vehicle.airspeed = speed
    destination = LocationGlobalRelative(longitude, latatude, height)
    vehicle.simple_goto(destination)

    # Sets previous cordinates
    previous_lon = vehicle.location.global_relative_frame.lat
    previous_lat = vehicle.location.global_relative_frame.lon
    sleep(2)

    # Loop to check the current cordinates to previous cordinates to identify when the drone is no longer moving (ie. at destination)
    while True:
        vehicle_lon = vehicle.location.global_relative_frame.lat
        vehicle_lat = vehicle.location.global_relative_frame.lon

        # Does comparison of current and previous cordinates if the same then break else set previous cordinates to current cordinates and check again in 2 seconds
        if round(vehicle_lon, 5) == round(previous_lon, 5) and round(vehicle_lat, 5) == round(previous_lat, 5):
            break
        else:
            previous_lon = vehicle_lon
            previous_lat = vehicle_lat
            sleep(2)

# Function that uses the others to make multiple waypoints simple to write
def mission(vehicle, destination, height, speed):
    # Change mode to guided
    while vehicle.mode != "GUIDED":
        sleep(1)
        
    # Arm the drone
    while not vehicle.armed:
        vehicle.armed = True
        sleep(1)

    # Go up to target height
    launch(vehicle, height)

    # Fly to destination
    go_to_destination(vehicle, destination, height, speed)

    # Close current conneciton
    vehicle.close()

# Log date and time
filewrite(file, str(date_time))

# Fly to first location
vehicle = connectMyCopter()
home_cordinates = identify_home()

# Change mode to guided
while vehicle.mode != "GUIDED":
    sleep(1)
        
st = time.time()
        
while True:
    # Fly to waypoint1
    mission(vehicle, waypoint1, height, speed)

    # Fly home
    mission(vehicle, home_cordinates, height, speed)

    # Rate of checks
    while True:
        if vehicle.mode != "GUIDED":
            et = time.time()
            elapsed_time = round(((et - st)/60), 5)
            break
        else:
            sleep(2)
        
filewrite(file, "Elapsed_Time: {} min".format(elapsed_time))