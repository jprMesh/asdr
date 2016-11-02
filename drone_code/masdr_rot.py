'''
Rotation pattern script for 3DR Solo.
I believe this will work, at least as far as connecting to an inflight drone.
To run, connect to the `solo wifi` network hosted by the controller and access
the Solo CLI. From the CLI, run this script by calling:
    solo script run masdr_rot.py
The drone should change to guided control (controller will not work for duration
of the maneuver), and begin rotating. After completing the rotation, it should
restore RC control to the controller.
'''

from dronekit import connect, VehicleMode
from time import sleep

target = 'udpin:0.0.0.0:14550'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

print "Starting rotation..."
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