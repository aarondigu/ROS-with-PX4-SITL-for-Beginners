#!/usr/bin/env python3
from pymavlink import mavutil, mavwp
import time
import math
import matplotlib.pyplot as plt
from drawnow import drawnow
import utm

# Start a connection listening to a UDP port
mav= mavutil.mavlink_connection('udpin:localhost:14550')
#mav = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

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


waypoints = [(0.0,0.0), (0.0,10.0), (10.0,10.0), (10.0,0.0), (0.0,0.0)]
#waypoints = [(0.0,0.0), (0.0,5.0), (5.0,5.0), (5.0,0.0), (10.0, 0.0), (10.0, 5.0), (15.0, 5.0), (15.0, 0.0), (0.0, 0.0)]
#waypoints = [(0.0,0.0), (0.0,20.0), (20.0,20.0), (20.0,0.0), (0.0,0.0)]
# waypoints = [(0.0,0.0), (0.0,20.0)]

home_location = waypoints[0]
seq = 0
for waypoint in enumerate(waypoints):
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    seq = waypoint[0]
    lat, lon = waypoint[1]
    altitude = 7 # 7 meter
    autocontinue = 1
    current = 0
    param1 = 0.0 # minimum pitch
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

# for i in range(wp.count()*10):
#     msg = mav.recv_match(type=['MISSION_REQUEST'],blocking=True,timeout=1)
#     mav.mav.send(wp.wp(msg.seq))
#     print ('Sending waypoint {0}'.format(msg.seq))    

while(True):
	try:
	    msg = mav.recv_match(type=['MISSION_REQUEST'],blocking=True, timeout = 0.5)
	    mav.mav.send(wp.wp(msg.seq))
	    print ('Sending waypoint {0}'.format(msg.seq))
	except:
		break

msg = mav.recv_match(type=['MISSION_ACK'],blocking=True, timeout = 0.5) # OKAY
try:
	print(msg.type)
except:
	print("No ACK received")

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
				vel = math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)
				v.append(vel)
				vx.append(msg.vx)
				vy.append(msg.vy)
				vz.append(-msg.vz)
				print("Velocidad: %f" % vel)
			else:
			 	print("No LOCAL_POSITION_NED message yet")
			# DEBUG_VECT message
			msg = m.recv_match(type='DEBUG_VECT', blocking=True, timeout = 0.15)
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
				#print("No DEBUG_VECT message yet")
				time.sleep(0.05)
			time.sleep(0.05)

		except KeyboardInterrupt:
			break


xd = list() #posicion x dron
yd = list() #posicion y dron
v = list()	#velocidad dron
vx = list()	#velocidad dron
vy = list()	#velocidad dron
vz = list()	#velocidad dron
t = list()	#tiempo 

read_loop(mav)

figure, axes = plt.subplots()
# plt.axis([-5, 40, -5, 30]) 
# Add title and axis names
plt.title('Mision: buscar ganado hectarea')
plt.xlabel('x [m] (Este)')
plt.ylabel('y [m] (Norte)')
plt.plot(xd,yd, label='Recorrido del dron')


# Ruta con obstaculos
obs1 = plt.Circle((0,7), 0.5, color ='r')
obs2 = plt.Circle((-1.2,7), 0.5, color ='r') 
obs3 = plt.Circle((-3,12), 0.5, color ='r') 
obs4 = plt.Circle((1,12), 0.5, color ='r') 
obs5 = plt.Circle((10,20), 0.5, color ='r') 
obs6 = plt.Circle((10, 18.8), 0.5, color ='r') 
obs7 = plt.Circle((7,0), 0.5, color ='r') 
obs8 = plt.Circle((7, -1.2), 0.5, color ='r') 
obs9 = plt.Circle((19.8,10), 0.5, color ='r') 
obs10 = plt.Circle((21.2, 10), 0.5, color ='r') 

# Estresar el sistema
# obs1 = plt.Circle((7, 0), 0.5, color ='r')
# obs2 = plt.Circle((7, 1.2), 0.5, color ='r') 
# obs3 = plt.Circle((7, -1.2), 0.5, color ='r') 
# obs4 = plt.Circle((7, 2.4), 0.5, color ='r') 
# obs5 = plt.Circle((7, -2.4), 0.5, color ='r') 
# obs6 = plt.Circle((7, 3.6), 0.5, color ='r') 
# obs7 = plt.Circle((7, -3.6), 0.5, color ='r') 

# #axes.set_aspect(1)
axes.add_artist(obs1)
axes.add_artist(obs2)
axes.add_artist(obs3)
axes.add_artist(obs4)
axes.add_artist(obs5)
axes.add_artist(obs6)
axes.add_artist(obs7)
axes.add_artist(obs8)
axes.add_artist(obs9)
axes.add_artist(obs10)

xwp, ywp = zip(*waypoints)
plt.plot(xwp,ywp, '--', label='Ruta deseada')

plt.legend()

plot2 = plt.figure(2)
# Add title and axis names
plt.title('Velocidad del dron en la mision')
plt.xlabel('t [s]')
plt.ylabel('v [m/s]')
plt.plot(t, v, label="Magnitud de velocidad", linewidth= 5.0)
plt.plot(t, vx, label="Velocidad en x")
plt.plot(t, vy, label="Velocidad en y")
plt.plot(t, vz, label="Velocidad en z")
plt.plot([0, t[len(t)-1]], [3, 3], '--', label='Magnitud de velocidad maxima' )
plt.legend()
plt.show()
