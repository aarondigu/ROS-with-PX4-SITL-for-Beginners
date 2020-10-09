#!/usr/bin/env python3

from pymavlink import mavutil
import time

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))

# Once connected, use 'the_connection' to get and send messages

def read_loop(m):

	messages = ['DEBUG_VECT', 'HEARTBEAT', 'LOCAL_POSITION_NED']

	while(True):

		for i in messages:
		# grab a mavlink message
			msg = m.recv_match(type=i, blocking=True)
			if  msg:
				# handle the message based on its type
				msg_type = msg.get_type()
				if msg_type == "BAD_DATA":
					if mavutil.all_printable(msg.data):
						sys.stdout.write(msg.data)
						sys.stdout.flush()
				else: 
					print("%s: %s" % (i, msg))
			else:
				print("No message yet")
		time.sleep(1)


#msg = the_connection.recv_match(type='DEBUG_VECT',blocking=True)

read_loop(the_connection)