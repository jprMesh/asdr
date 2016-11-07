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

drone = None

def main():
    target = 'udpout:0.0.0.0:14560'
    #target = 'udpin:0.0.0.0:14550'
    print 'Connecting to ' + target + '...'
    drone = connect(target, wait_ready=True)

    while 1:
        if not drone.armed:
            continue
        command = raw_input('MASDR>$ ')
        if command == 'rotate':
            rotate()
        if command == 'exit':
            break
    drone.close()

def rotate():
    """ Take control of drone and command it to rotate 360 degrees """
    print 'Starting rotation...'
    chmode('GUIDED')
    # Go to North
    # condition_yaw(0, relative=False)
    # sleep(3)
    condition_yaw(90, relative=True)
    sleep(1.5)
    condition_yaw(90, relative=True)
    sleep(1.5)
    condition_yaw(90, relative=True)
    sleep(1.5)
    condition_yaw(90, relative=True)
    sleep(1.5)
    chmode('LOITER')

def chmode(new_mode):
    """ Change drone mode (guided, loiter, etc) """
    drone.mode = VehicleMode(new_mode)
    while drone.mode.name != new_mode:
        drone.mode = VehicleMode(new_mode)
        sleep(0.1)

def condition_yaw(heading, relative=False):
    """ Send command to yaw a certain amount """
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = drone.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to drone
    drone.send_mavlink(msg)

if __name__ == '__main__':
    main()
