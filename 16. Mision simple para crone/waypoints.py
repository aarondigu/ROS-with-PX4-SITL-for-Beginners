#!/usr/bin/env python3

import math
import utm


waypoints = [(19.638275, -99.116289), (19.638400, -99.116293)]

# Primero los puntos estan en NED (x: norte, y: este), el YAW se mide desde el norte, positivo hacia el este

yaw_NED = 0
yaw_NED = yaw_NED*math.pi/180.0 # convierte de grados a radianes

yaw_ENU = -yaw_NED + math.pi/2
if (yaw_ENU > math.pi):
	yaw_ENU -= 2*math.pi

yaw = yaw_ENU
yaw_ENU = yaw_ENU*180/math.pi
print(yaw_ENU)


R = [[math.cos(yaw), -math.sin(yaw)],[math.sin(yaw), math.cos(yaw)]]
#print(R)

wp_XY = (5, 0);
wp_ENU = (wp_XY[0] * R[0][0] + wp_XY[1] * R[0][1], wp_XY[0] * R[1][0] + wp_XY[1] * R[1][1])
print(wp_ENU)

for waypoint in enumerate(waypoints):
	if (waypoint[0] == 0):
		lat,lon = waypoint[1]
		utm_wp = utm.from_latlon(lat,lon)
		x0 = utm_wp[0] #Easting
		y0 = utm_wp[1] #Northing
	else:
		lat,lon = waypoint[1]
		utm_wp = utm.from_latlon(lat,lon)
	x = utm_wp[0] - x0
	y = utm_wp[1] - y0
	print("x: ", x, ", y: ", y) # x: este, y: norte