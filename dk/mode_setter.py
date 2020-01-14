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

