from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from gpiozero import AngularServo
from time import sleep
import argparse
from datetime import datetime

date_time = str(datetime.now())
file = "Flight_Log_Auto.txt"

# Destination cordinates
waypoint1 = [38.403390, -121.398838]

# Height and speed parameters for autonomus flight
height = 17
speed = 10

# Set up servo
servo = AngularServo(12, min_angle= 0, max_angle= 270,
                     min_pulse_width= 0.0005,
                     max_pulse_width= 0.0025)

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
    filewrite(file, "Successful Connection")
    return vehicle

# Funciton to launch
def launch(vehicle, height):
    vehicle.simple_takeoff(height)

    while True:
        print("Alt: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= height * .95:
            print("Reahced target altitude")
            filewrite(file, "Target Altitude Reached")
            break
        else:
            sleep(1)

# Function to go to cordinates
def go_to_destination(vehicle, destination, height, speed):
    # Split longitude and latatude from destination list
    filewrite(file, "Going to Destination")
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
    destination_acheived = False
    while not destination_acheived:
        vehicle_lon = vehicle.location.global_relative_frame.lat
        vehicle_lat = vehicle.location.global_relative_frame.lon

        # Does comparison of current and previous cordinates if the same then break else set previous cordinates to current cordinates and check again in 2 seconds
        if round(vehicle_lon, 5) == round(previous_lon, 5) and round(vehicle_lat, 5) == round(previous_lat, 5):
            filewrite(file, "Destination Reached")
            destination_acheived = True
        else:
            filewrite(file, "Previous: {} {} Current: {} {}".format(previous_lon, previous_lat, vehicle_lon, vehicle_lat))
            previous_lon = vehicle_lon
            previous_lat = vehicle_lat
            sleep(2)

# Function that uses the others to make multiple waypoints simple to write
def mission(vehicle, destination, height, speed):
    # Change mode to guided
    while vehicle.mode != "GUIDED":
        print("Waiting for vehicle to be in GUIDED mode")
        sleep(1)
        
    # Arm the drone
    while not vehicle.armed:
        filewrite(file, "Attempting to Arm")
        vehicle.armed = True
        sleep(1)

    filewrite(file, "Armed and Taking Off")

    # Go up to target height
    filewrite(file, "Fly to Cruising Height")
    launch(vehicle, height)

    # Fly to destination
    go_to_destination(vehicle, destination, height, speed)
    filewrite(file, "Flying to Selected Locaiton")

    # Land at destination
    filewrite(file, "Attempting to Land at Selected Location")
    vehicle.mode = VehicleMode("LAND")

    # Checks altitude so it is known when land occured
    while True:
        print("Alt: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 1 * .95:
            filewrite(file, "Landed")
            break
        else:
            sleep(1)

    # Close current conneciton
    vehicle.close()

# Log date and time
filewrite(file, str(date_time))

# Fly to first location
vehicle = connectMyCopter()
print("About to takeoff...")
home_cordinates = identify_home()
mission(vehicle, waypoint1, height, speed)

# Drop Package
servo.angle = 260
sleep(5)

# Fly home
vehicle_home = connectMyCopter()
print("About to takeoff...")
vehicle_home.mode = VehicleMode("GUIDED")
mission(vehicle_home, home_cordinates, height, speed)

# End of script
filewrite(file, "End of Script")

# Rate of checks and keep package servo locked
while True:
    servo.angle = 0
    sleep(2)