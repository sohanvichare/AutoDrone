import numpy as np
import math
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
from dronekit_sitl import SITL
from imutils.object_detection import non_max_suppression



#defines drone variable
vehicle = connect("/dev/ttyACM0", wait_ready=True);
#defines variable t
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())



#define location class that stores latitude and longitude
class Location:
   "Basic Location class, used to store latitude and longitude"

   def __init__(self, latitude, longitude):
      self.latitude = latitude
      self.longitude = longitude


#define SafeLocation class, which stores location data, which people are at each Safe Location
class SafeLocation:
    "Safe Location class, used to store data for a safe location. Has two attributes, one is location (a location object which stores the location), and the second is peopleArrivedArray, which is an array that stores the people who have reached this location."

    def __init__(self, location):
        self.location = location
        #an array of Person objects, used to store what people are already at the safeLocation
        self.peopleArrivedArray = [];

#define helper method to print arrays with locations so that they are easy to see
def printLocationArray(locationArray):
    for item in locationArray:
        print item.latitude
        print item.longitude
        print " "

#define helper method to get an array of the locations object from an array of a person, safelocation, or any other object that has location as a property
def getLocationsArray(safeLocsArray):
        returnArray = []
        for item in safeLocsArray:
            returnArray.append(item.location);
        return returnArray;
        
#distance formula function, finds distance between two location objects, accounting for the earth's spherical shape (assumes that earth is a sphere)
def distanceBetweenLocations(location1, location2):
 
    # Convert latitude and longitude to spherical coordinates in radians.
    degreesToRadians = math.pi/180.0
         
    # phi = 90 - latitude
    phi1 = (90.0 - location1.latitude)*degreesToRadians
    phi2 = (90.0 - location2.latitude)*degreesToRadians
         
    # theta = longitude
    theta1 = location1.longitude*degreesToRadians
    theta2 = location2.longitude*degreesToRadians
         
    # Compute spherical distance from spherical coordinates.
         
    # For two locations in spherical coordinates 
    # (1, theta, phi) and (1, theta', phi')
    # cosine( arc length ) = 
    #    sin phi sin phi' cos(theta-theta') + cos phi cos phi'
    # distance = rho * arc length
     
    cos = (math.sin(phi1)*math.sin(phi2)*math.cos(theta1 - theta2) + 
           math.cos(phi1)*math.cos(phi2))
    arc = math.acos( cos )
 
    # multiply and return arc by the right distance unit, miles or kilometers
    unit = 1;
    
    return arc * unit;

def closestSafePoint(droneLocation, safePointLocationArray):
        #define array that will be returned
        closestPointIndexArray = [];
        usedIndexArray = [];
        closestDist = 1000;
        closestIndex = 0;
        
        for index, item in enumerate(safePointLocationArray):
            for ind, itm in enumerate(safePointLocationArray):
                if ind not in usedIndexArray:
                    dist = distanceBetweenLocations(itm.location, droneLocation);
                    if dist <= closestDist:
                        closestDist = dist;
                        closestIndex = ind;
                        
            if closestIndex not in usedIndexArray:
                usedIndexArray.append(closestIndex);
                closestPointIndexArray.append(closestIndex);
                closestDist = 1000;

        return closestPointIndexArray.sort();

#takes a "picture" of what the picamera is seeing by saving the array
def takePicture():
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    return image


# gets the number of people in the image
def readImage(image):
    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
	padding=(8, 8), scale=1.05)
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
    count=0
    for (xA, yA, xB, yB) in pick:
	    count=count+1
    return count

def findNumPeople():
    img =takePicture();
    people= readImage(img)
    return people
'''dx = R*cos(theta) 
   = 500 * cos(135 deg) 
   = -353.55 meters

dy = R*sin(theta) 
   = 500 * sin(135 deg) 
   = +353.55 meters

delta_longitude = dx/(111320*cos(latitude)) 
                = -353.55/(111320*cos(41.88592 deg))
                = -.004266 deg (approx -15.36 arcsec)

delta_latitude = dy/110540
               = 353.55/110540
               =  .003198 deg (approx 11.51 arcsec)

Final longitude = start_longitude + delta_longitude
                = -87.62788 - .004266
                = -87.632146

Final latitude = start_latitude + delta_latitude
               = 41.88592 + .003198
               = 41.889118
'''

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


#initializes drone
#connection string is the port at which the rpi connects to drone
#isSimulator is a boolean value for whether this is being used on the actual drone or a simulator
def initializeDrone(connectionString, isSimulator):
    if isSimulator:
        sitl = SITL()
        sitl.download('copter', '3.3', verbose=True)
        sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
        sitl.launch(sitl_args, await_ready=True, restart=True)
        print "Connecting to vehicle on: 'tcp:127.0.0.1:5760'"
        vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
        # print information about vehicle
        print "Global Location: %s" % vehicle.location.global_frame
        print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
        print "Local Location: %s" % vehicle.location.local_frame    #NED
        print "Attitude: %s" % vehicle.attitude
        print "Velocity: %s" % vehicle.velocity
        print "GPS: %s" % vehicle.gps_0
        print "Groundspeed: %s" % vehicle.groundspeed
        print "Airspeed: %s" % vehicle.airspeed
        print "Battery: %s" % vehicle.battery
        print "EKF OK?: %s" % vehicle.ekf_ok
        print "Last Heartbeat: %s" % vehicle.last_heartbeat
        print "Rangefinder: %s" % vehicle.rangefinder
        print "Rangefinder distance: %s" % vehicle.rangefinder.distance
        print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
        print "Heading: %s" % vehicle.heading
        print "Is Armable?: %s" % vehicle.is_armable
        print "System status: %s" % vehicle.system_status.state
        print "Mode: %s" % vehicle.mode.name    # settable
        print "Armed: %s" % vehicle.armed
    else:
        # Connect to the Vehicle
        print 'Connecting to vehicle;'
        vehicle = connect(connectionString, wait_ready=True)
        # print information about vehicle
        print "Global Location: %s" % vehicle.location.global_frame
        print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
        print "Local Location: %s" % vehicle.location.local_frame    #NED
        print "Attitude: %s" % vehicle.attitude
        print "Velocity: %s" % vehicle.velocity
        print "GPS: %s" % vehicle.gps_0
        print "Groundspeed: %s" % vehicle.groundspeed
        print "Airspeed: %s" % vehicle.airspeed
        print "Battery: %s" % vehicle.battery
        print "EKF OK?: %s" % vehicle.ekf_ok
        print "Last Heartbeat: %s" % vehicle.last_heartbeat
        print "Rangefinder: %s" % vehicle.rangefinder
        print "Rangefinder distance: %s" % vehicle.rangefinder.distance
        print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
        print "Heading: %s" % vehicle.heading
        print "Is Armable?: %s" % vehicle.is_armable
        print "System status: %s" % vehicle.system_status.state
        print "Mode: %s" % vehicle.mode.name    # settable
        print "Armed: %s" % vehicle.armed

#flies a vehicle to a target altitude
def arm_and_takeoff(aTargetAltitude):
    #Arms vehicle and fly to aTargetAltitude.

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude)
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

#stops program and counts down for a certain number of seconds
def countdown(amtTime):
    i = 0
    while i <= amtTime:
        print("COUNTDOWN: "+str(amtTime-i))
        time.sleep(1)
        i = i+1

#stops program and countrs down for a certain number of seconds while displaying the altitude
def countdownAlt(amTime):
    o = 0
    while o <= amTime:
        print("COUNTDOWN: "+str(amTime-o))
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        time.sleep(1)
        o = o+1

#initializes and takes off drone
#alt: alitutude in meters
#countdownSeconds = number of seconds till takeoff
#isSim = true if we are using a simulator
#connectString = port at which the rpi is connected to drone "/dev/tty/ACM0" for usb
def takeOff(alt, countdownSeconds, isSim, connectString):
    initializeDrone("/dev/tty/ACM0", isSim)
    countdown(countdownSeconds)
    vehicle.airspeed=3
    arm_and_takeoff(alt)
    
def moveVehicle(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

#goto helper function for drone
def goToLocation(targetLocation, gotoFunction=vehicle.simple_goto):
    currentLocation=vehicle.location.global_relative_frame
    targetDistance=get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
        print "Distance to target: ", remainingDistance
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print "Reached target"
            break;
        time.sleep(2)



########ACTUAL PROGRAM STARTS HERE ALL FUNCTIONS GO ABOVE THIS############
safePointArray = [SafeLocation(Location(0,0)), SafeLocation(Location(0,0)), SafeLocation(Location(0,0)), SafeLocation(Location(0,0))]
squareLength = input("Input side length of square to survey in meters: ")
takeOff(5,5,True,"/dev/tty/AMA0")
# Set airspeed using attribute
vehicle.airspeed = 1 #m/s
# Set groundspeed using attribute
vehicle.groundspeed = 1 #m/s
homeL = vehicle.home_location
homeLat = homeL.lat
homeLong = homeL.lon
home = Location(homeLat, homeLong)
currLoc = "";
numFiveMeterSegments = int(round(squareLength/5))
trackFiveMeterSegments = numFiveMeterSegments
for i in range(0,numFiveMeterSegments):
    #move the vehicle north for 5 seconds at a speed of 1 m/s
    moveVehicle(1,0,0,5)
    #update currLoc variable to store location
    currLoc = Location(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon)
    #not necessary lmfaooooo
    trackFiveMeterSegments = trackFiveMeterSegments - 1
    #finds num niggas 
    numPeople = findNumPeople()
    if numPeople >= 1:
        currLoc = Location(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon)
        index = closestSafePoint(currLoc, safePointArray)
        safePointLocation = LocationGlobalRelative(safePointArray[index[0]].location.latitude, safePointArray[index[0]].location.longitude, 5)
        goToLocation(safePointLocation)
        time.sleep(5)
        goToLocation(currLoc)
    
    if trackFiveMeterSegments == 0:
        break;
