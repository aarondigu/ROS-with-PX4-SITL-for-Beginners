#!/usr/bin/env python3
from pymavlink import mavutil, mavwp
import time
import math
import matplotlib.pyplot as plt
from drawnow import drawnow
import utm

# Start a connection listening to a UDP port
mav= mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
mav.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (mav.target_system, mav.target_system))

# Make Waypoints
wp = mavwp.MAVWPLoader()

#Get home function, lat/long are returned *10^7
def cmd_get_home():
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = mav.recv_match(type=['COMMAND_ACK'],blocking=True)
    print (msg)
    msg = mav.recv_match(type=['HOME_POSITION'],blocking=True)
    return (msg.latitude/10000000.0, msg.longitude/10000000.0, msg.altitude/10000000.0)

mes = cmd_get_home()
print(mes)
# (northing, easting)
waypoints = [(mes[0],mes[1]), (mes[0]+.0002,mes[1]), (mes[0]+.0002,mes[1]+.0001), (mes[0],mes[1]+.0001), (mes[0],mes[1]+.0002), (mes[0]+.0002,mes[1]+.0002), (mes[0]+.0002,mes[1]+.0003), (mes[0],mes[1]+.0003), (mes[0],mes[1])]
home_location = waypoints[0]
seq = 0
for waypoint in enumerate(waypoints):
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    seq = waypoint[0]
    lat, lon = waypoint[1]
    altitude = 7 # 7 meter
    autocontinue = 1
    current = 0
    param1 = 15.0 # minimum pitch
    if seq == 0: # first waypoint to takeoff
        current = 1
        p = mavutil.mavlink.MAVLink_mission_item_message(mav.target_system, mav.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, current, autocontinue, param1, 0, 0, float("nan"), lat, lon, altitude)
    elif seq == len(waypoints) - 1: # last waypoint to land
        p = mavutil.mavlink.MAVLink_mission_item_message(mav.target_system, mav.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_LAND, current, autocontinue, 0, 0, 0, float("nan"), lat, lon, altitude)
    else:
        p = mavutil.mavlink.MAVLink_mission_item_message(mav.target_system, mav.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, current, autocontinue, 0, 0, 0, float("nan"), lat, lon, altitude)
    wp.add(p)

# Send Waypoints to airframe
mav.waypoint_clear_all_send()
mav.waypoint_count_send(wp.count())

for i in range(wp.count()):
    msg = mav.recv_match(type=['MISSION_REQUEST'],blocking=True)
    mav.mav.send(wp.wp(msg.seq))
    print ('Sending waypoint {0}'.format(msg.seq))    

msg = mav.recv_match(type=['MISSION_ACK'],blocking=True) # OKAY
print(msg.type)

def read_loop(m):
	count = 0
	while(True):
		try: 
			# LOCAL_POSITION_NED message
			msg = m.recv_match(type='LOCAL_POSITION_NED', blocking=False)
			if  msg:
				if count == 0:
						t0 = msg.time_boot_ms
						count = 1
				mt = (msg.time_boot_ms - t0)/1000.0
				t.append(mt)

				print("LOCAL_POSITION_NED: ")
				print("x:    %f" % msg.x)
				print("y:    %f" % msg.y)
				print("z:    %f" % msg.z)
				xd.append(msg.y)
				yd.append(msg.x) #NED to ENU
				# MPC_XY_CRUISE = 3.0, maximum horizontal velocity of 3 m/s
				vel = math.sqrt(msg.vx**2 + msg.vz**2 + msg.vz**2)
				v.append(vel)
				print("Velocidad: %f" % vel)
			else:
			 	print("No LOCAL_POSITION_NED message yet")
			# DEBUG_VECT message
			msg = m.recv_match(type='DEBUG_VECT', blocking=True, timeout = 0.05)
			if  msg:
					print(msg)
					if (msg.x != 0 and msg.y != 0):
						split = msg.name.split()
						index = int(split[1])
						tc[index] = mt
						xc[index] = msg.x
						yc[index] = msg.y
						nc[index] = msg.name
			else:
				print("No DEBUG_VECT message yet")
			time.sleep(0.05)

		except KeyboardInterrupt:
			break

xd = list() #posicion x dron
yd = list() #posicion y dron
xc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]#posicion x vaca
yc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #posicion y vaca
nc = ["", "", "", "", "", "", "", ""] #id vaca
v = list()	#velocidad dron
t = list()	#tiempo 
tc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]#tiempo vaca

read_loop(mav)

plot1 = plt.figure(1)
plt.axis([-5, 40, -5, 30]) 
# Add title and axis names
plt.title('Mision: buscar ganado con viento de 10 m/s')
plt.xlabel('x [m] (Este)')
plt.ylabel('y [m] (Norte)')
plt.plot(xd,yd, label='Recorrido del dron')
for i in range(len(xc)):
	plt.scatter(xc[i],yc[i],color='red')
	plt.annotate(nc[i] + "\n t: " + str(tc[i]) + " s",(xc[i],yc[i]))

#plot waypoints
xwp = list()
ywp = list()
for waypoint in enumerate(waypoints):
	if (waypoint[0] == 0):
		lat,lon = waypoint[1]
		utm_wp = utm.from_latlon(lat,lon)
		x0 = utm_wp[0] #Easting
		y0 = utm_wp[1] #Northing
	else:
		lat,lon = waypoint[1]
		utm_wp = utm.from_latlon(lat,lon)
	x = utm_wp[0]
	y = utm_wp[1]

	xwp.append(x-x0)
	ywp.append(y-y0)

plt.plot(xwp,ywp, '--', label='Ruta deseada')
plt.legend()

plot2 = plt.figure(2)
# Add title and axis names
plt.title('Velocidad del dron en la mision con viento de 10 m/s')
plt.xlabel('t [s]')
plt.ylabel('v [m/s]')
plt.plot(t, v, label="Velocidad")
plt.plot([0, 80], [3, 3], '--', label='Velocidad maxima' )
plt.legend()
plt.show()