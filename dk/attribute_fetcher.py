##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse


#########FUNCTIONS#################

def connectMyCopter():

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect

	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()

	vehicle = connect(connection_string,wait_ready=True)

	return vehicle



##########MAIN EXECUTABLE###########

vehicle = connectMyCopter()

#Version and attributes
vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s'%vehicle.version)

#Does the firmware support the companion pc to set the attitude
print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)

#Read actual position
print('Position: %s'%vehicle.location.global_relative_frame)

#Read the actual attitude roll, pitch, yaw
print('Attitude: %s'%vehicle.attitude)

#Read the actual velocity (m/s)
print('Velocity: %s'%vehicle.velocity) #NED: North East Down convention

#When did we receive last heartbeat
print('Last Heartbeat: %s'%vehicle.last_heartbeat)

#Is the vehicle good to arm
print('Is the vehicle armable: %s'%vehicle.is_armable)

#What is total groundspeed
print('Groundspeed: %s'%vehicle.groundspeed) #This is settable

#What is the actual flight mode
print('Mode: %s'%vehicle.mode.name) 	     #This is settable

#Is the vehicle armed
print('Armed: %s'%vehicle.armed) 	     #This is settable

#Is state estimation filter ok
print('EKF Ok: %s'%vehicle.ekf_ok)

vehicle.close()

