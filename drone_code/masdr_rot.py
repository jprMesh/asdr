'''
Rotation pattern script for 3DR Solo.
I believe this will work, at least as far as connecting to an inflight drone.
Before running, while still connected to the internet, the script has to be
"packed up" with all the dependencies by calling the 'pack' utility:
        solo script pack
This will create a file called 'solo-script.tar.gz', which I think is what
actually gets uploaded in the next step.
To run, connect to the 'solo wifi' network hosted by the controller and access
the Solo CLI. From the CLI, run this script by calling:
        solo script run masdr_rot.py
The drone should change to guided control (controller will not work for duration
of the maneuver), and begin rotating. After completing the rotation, it should
restore RC control to the controller.
'''

from dronekit import connect, VehicleMode, mavutil
from time import sleep

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

target = 'udpout:0.0.0.0:14560'
#target = 'udpin:0.0.0.0:14550'
print 'Connecting to ' + target + '...'
vehicle = connect(target, wait_ready=True)

print "Starting rotation..."
while vehicle.mode.name != "GUIDED":
    vehicle.mode = VehicleMode("GUIDED")
    sleep(0.5)
condition_yaw(0, relative=False) # Go to North
sleep(0.5)
condition_yaw(90, relative=False)
condition_yaw(180, relative=False)
condition_yaw(270, relative=False)
condition_yaw(0, relative=False)
while vehicle.mode.name != "LOITER":
    vehicle.mode = VehicleMode("LOITER")
    sleep(0.5)
vehicle.close()
print "Done."