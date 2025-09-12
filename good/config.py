import socket
import math

hostname = socket.gethostname()

if hostname == 'storm':
    print('running on taj\'s bot')
    # front left: 26, front right: 27, back right: 29, back left: 25
    motor_addresses = [25, 29, 26, 27]
    motor_addresses_named = {'fl': 26, 'fr': 27, 'bl': 25, 'br': 29}
    # idk if these get used
    # SINCOSCENTRE = {25: 1260, 26: 1250, 27: 1219, 29: 1256}
    # ELECANGLEOFFSET = {25: 1452253440, 26: 1167990784, 27: 1478825728, 29: 1547735552}
    """ tof stuff """
    tof_count = 8
    tof_addresses = [80, 88, 94, 91, 95, 93, 92, 90]
    tof_angles = { # address -> radians, from north, counter-clockwise
        80: 0,
        90: math.radians(55),
        92: math.radians(90),
        93: math.radians(125),
        95: math.radians(180),
        91: math.radians(215),
        94: math.radians(270),
        88: math.radians(305),
    }
    tof_offsets = { # address -> offset in mm from center of bot
        80: 42.5,
        95: 42.5,
        92: 57,
        94: 57,
        90: 111.5,
        93: 111.5,
        91: 111.5,
        88: 111.5,
    }
else:
    print(f'running on {hostname}, please provide a configuration for this bot')