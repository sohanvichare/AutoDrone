import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from dronekit_sitl import SITL

#defines variable t
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera)

#takes a "picture" of what the picamera is seeing by saving the array
def takePicture():
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    return image


# gets the number of people in the image
def readImage(image):
    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    count=0
    for (x,y,w,h) in faces:
        count=count+1
    return count

def findNumPeople():
    img =takePicture();
    people= readImage(img)
    return people
    
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
def initAndTakeOff(alt):
    
    
