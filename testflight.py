from dronekit import connect
vehicle = connect("/dev/ttyACM0", wait_ready=True)

# vehicle is an instance of the Vehicle class
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

def countdown(amtTime):
    i = 0
    while i <= amtTime:
        print("COUNTDOWN: "+str(amtTime-i))
        time.sleep(1)
        i = i+1

def countdownAlt(amTime):
    o = 0
    while o <= amTime:
        print("COUNTDOWN: "+str(amTime-o))
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        time.sleep(1)
        o = o+1

countdown(5)
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
home = vehicle.home_location
arm_and_takeoff(1);
vehicle.airspeed=3
vehicle.simple_goto(home)
countdownAlt(30);
print("Done")
