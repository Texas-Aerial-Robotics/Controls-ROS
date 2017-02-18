print "Start simulator (SITL)"
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

#import DroneKit-Python
from dronekit import connect, VehicleMode

#Connect to vehicle
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

#Get vehicle attributes
print "Getting vehicle attribute values:"
print "GPS: %s" % vehicle.gps_0
print "Battery: %s" % vehicle.battery
print "Last Heartbeat: %s" % vehicle.last_heartbeat
print "Is Armable?: %s" % vehicle.is_armable
print "System status: %s" % vehicle.system_status.state
print "Mode: %s" % vehicle.mode.name

#close vehicle before exiting script
vehicle.close()

#shut down sim
sitl.stop()
print("Completed")