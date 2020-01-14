##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException,Command
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

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

def arm_and_takeoff(targetHeight):
	while vehicle.is_armable!=True:
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE. Have fun!!")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Waiting for vehicle to become armed.")
		time.sleep(1)
	print("Look out! Virtual props are spinning!!")

	vehicle.simple_takeoff(targetHeight) ##meters

	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached!!")

	return None


##########MAIN EXECUTABLE###########

vehicle = connectMyCopter()

##Command template
#Command(0,0,0,FrameOfReference,MAVLinkCommand,CurrentWP,AutoContinue,param1,param2,param3,param4,param5,parm6,param7)

wphome=vehicle.location.global_relative_frame

##List of commands
cmd1=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,wphome.lat,wphome.lon,wphome.alt)
cmd2=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,44.501375,-88.062645,15)
cmd3=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,44.501746,-88.062242,10)
cmd4=Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0)

##Donload current list of commands FROM the drone we're connected to
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

##Clear the current list of commands
cmds.clear()

##Add in our new commands
cmds.add(cmd1)
cmds.add(cmd2)
cmds.add(cmd3)
cmds.add(cmd4)

##Upload our commands to the drone
vehicle.commands.upload()

arm_and_takeoff(10)

print("After arm and takeoff")
vehicle.mode = VehicleMode("AUTO")
while vehicle.mode!="AUTO":
	time.sleep(.2)

while vehicle.location.global_relative_frame.alt>2:
	print("Drone is executing mission, but we can still run code")
	time.sleep(2)


