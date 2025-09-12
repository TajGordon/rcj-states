import socket
import math

hostname = socket.gethostname()
# motor addresses is: [back left, back right, front left, front right]

if hostname == 'storm':
    print('running on taj\'s bot')
    # front left: 26, front right: 27, back right: 29, back left: 25
    motor_addresses = [25, 29, 26, 27]
    # idk if these get used
    # SINCOSCENTRE = {25: 1260, 26: 1250, 27: 1219, 29: 1256}
    # ELECANGLEOFFSET = {25: 1452253440, 26: 1167990784, 27: 1478825728, 29: 1547735552}
    """ tof stuff """
    tof_count = 8
    tof_addresses = [80, 88, 94, 91, 95, 93, 92, 90]
    tof_enabled = {80: True, 88: True, 94: True, 91: True, 95: False, 93: True, 92: True, 90: True} # my back one is covered so disable it
    tof_angles = { # address -> radians, radians from north, starting at 0 and going clockwise
        80: 0, 
        88: math.radians(35), 
        94: math.radians(90), 
        91: math.radians(125), 
        95: math.radians(180), 
        93: math.radians(215), 
        92: math.radians(270), 
        90: math.radians(305)
    } 
    tof_offsets = { # address -> offset, in mm, like how far the tof is from the center of the robot
        80: 42.5,
        88: 111.5,
        94: 57,
        91: 111.5,
        95: 42.5,
        93: 111.5,
        92: 57,
        90: 111.5
    }
    """ localization stuff """
    localization_step_sizes = [10, 7, 5, 5, 3, 3, 2, 2, 1, 1, 1] # sorted in order that they will be searched, mm


else: # assuming siddak bot rn
    motor_addresses = [27, 28, 30, 26]
    print(f'running on {hostname}')