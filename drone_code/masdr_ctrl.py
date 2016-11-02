from dronekit import connect, VehicleMode
from sys import argv
from time import sleep

# Connect to UDP endpoint (and wait for default attributes to accumulate)
# So I don't know exactly how this works. I'm hoping we connect to the
#   controller, and then the controller sends our commands. If that's not the
#   case, then we have some thinking to do. We need to switch modes between
#   "guided", which lets the script control the drone, and "loiter", which lets
#   the controller control the drone. If we aren't controlling through the
#   controller, then idk how we communicate with the drone. If we get some USB
#   attachment that lets us talk, I still don't know how we negotiate control
#   with the controller.
target = sys.argv[1] if len(sys.argv) >= 2 else 'udpin:0.0.0.0:14550'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

print "Starting rotation"
while vehicle.mode.name != "GUIDED":
    vehicle.mode = VehicleMode("GUIDED")
    sleep(0.5)
vehicle.condition_yaw(0, relative=False) # Go to North
sleep(0.5)
vehicle.condition_yaw(90, relative=False)
vehicle.condition_yaw(180, relative=False)
vehicle.condition_yaw(270, relative=False)
vehicle.condition_yaw(0, relative=False)
while vehicle.mode.name != "LOITER":
    vehicle.mode = VehicleMode("LOITER")
    sleep(0.5)
vehicle.close()
print "Done."