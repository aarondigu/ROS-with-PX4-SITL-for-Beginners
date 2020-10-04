import time
from dronekit import connect

# Connect to the Vehicle (in this case a UDP endpoint)
vehicle = connect('127.0.0.1:14550', wait_ready=True)


@vehicle.on_message('DEBUG_VECT')
def listener(self, name, message):
    print "Name: %s" % message.name
    print "x: %s" % message.x
    print "y: %s" % message.y


# vehicle is an instance of the Vehicle class
print "Autopilot Firmware version: %s" % vehicle.version
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
print "Heading: %s" % vehicle.heading
print "System status: %s" % vehicle.system_status.state
print "Mode: %s" % vehicle.mode.name    # settable
print "Armed: %s" % vehicle.armed    # settable


while not vehicle.mode.name=='OFFBOARD' and not vehicle.armed:
    print " Getting ready to take off ..."
    time.sleep(1)

while 1:
	print "Mode: %s" % vehicle.mode.name    # settable
	print "Armed: %s" % vehicle.armed    # settable
	time.sleep(3)
