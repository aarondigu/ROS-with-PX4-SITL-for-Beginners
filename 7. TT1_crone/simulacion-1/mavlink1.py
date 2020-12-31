#!/usr/bin/env python3

from pymavlink import mavutil
import time
import matplotlib.pyplot as plt

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))

# Once connected, use 'the_connection' to get and send messages
def read_loop(m):
	count = 0
	while(True):
		try:
		# LOCAL_POSITION_NED message
			msg = m.recv_match(type='LOCAL_POSITION_NED', blocking=True)
			if  msg:
				print(msg)
				if (msg.z < -0.1 and msg.x > 0.1 and msg.y > 0.1):
					if (count == 0):
						t0 = msg.time_boot_ms
						count = 1
					mt = (msg.time_boot_ms - t0)/1000.0
					print('Time: %f' % mt)
					if (mt <= 30):
						ex.append(x - msg.x)
						ey.append(y - msg.y)
						ez.append(abs(z - msg.z))
						t.append(mt)
			else:
			 	print("No LOCAL_POSITION_NED message yet")
		except KeyboardInterrupt:
			break

x = 5
y = 5
z = -5

ex = list()
ey = list()
ez = list()
t = list()

read_loop(the_connection)

# Add title and axis names
plt.title('Error de posicion con viento de 10 m/s')
plt.xlabel('t [ms]')
plt.ylabel('e [m]')
plt.plot(t, ex, label='Error en x [m]')
plt.plot(t, ey, label='Error en y [m]')
plt.plot(t, ez, label='Error en z [m]')
plt.legend()
plt.show()
