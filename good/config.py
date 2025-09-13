import socket
import math

hostname = socket.gethostname()

if hostname == 'storm':
    print('running on taj\'s bot')
    target_goal = 'blue'
    # Motor addresses matching soccer_bot.py: [27-back left, 28-back right, 30-front left, 26-front right]
    # Motor addresses in order: [back_left, back_right, front_left, front_right]
    motor_addresses = [27, 28, 30, 26]  # [bl, br, fl, fr] - MATCHING SOCCER_BOT
    motor_addresses_named = {'fl': 30, 'fr': 26, 'bl': 27, 'br': 28}
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
    
    # Field dimensions (in mm) - RoboCup Junior field
    field_width = 2430  
    field_height = 1820  
    field_center_x = field_width / 2
    field_center_y = field_height / 2
    
    # Field boundaries (250mm from walls)
    field_boundary_left = 250
    field_boundary_right = field_width - 250
    field_boundary_top = 250
    field_boundary_bottom = field_height - 250
    
    # Goal dimensions (from official rules)
    goal_width = 450  # Internal width
    goal_depth = 74   # Internal depth (not 140mm as in Rust file)
    goal_crossbar_height = 140  # Crossbar height above playing surface
    goal_crossbar_depth = 20    # Maximum crossbar depth
    goal_center_y = field_height / 2  # Goal is centered vertically
    goal_distance_from_wall = 250  # Goal is flush with 250mm line from wall
    
    # Goal positions (left and right goals)
    goal_left_x = goal_distance_from_wall
    goal_right_x = field_width - goal_distance_from_wall
    
    # Goal exclusion zones (300mm in front of goal, 900mm wide)
    goal_exclusion_distance = 300
    goal_exclusion_width = 900
    goal_exclusion_half_width = goal_exclusion_width / 2
    
    # Goal exclusion zone boundaries
    goal_left_exclusion_x = goal_left_x + goal_depth + goal_exclusion_distance
    goal_left_exclusion_y_min = goal_center_y - goal_exclusion_half_width
    goal_left_exclusion_y_max = goal_center_y + goal_exclusion_half_width
    
    goal_right_exclusion_x = goal_right_x - goal_exclusion_distance
    goal_right_exclusion_y_min = goal_center_y - goal_exclusion_half_width
    goal_right_exclusion_y_max = goal_center_y + goal_exclusion_half_width
else:
    print(f'running on {hostname}, using default configuration')
    # Default configuration for development/testing
    # Motor addresses matching soccer_bot.py: [27-back left, 28-back right, 30-front left, 26-front right]
    # Motor addresses in order: [back_left, back_right, front_left, front_right]
    motor_addresses = [27, 28, 30, 26]  # [bl, br, fl, fr] - MATCHING SOCCER_BOT
    motor_addresses_named = {'fl': 30, 'fr': 26, 'bl': 27, 'br': 28}
    
    # Default TOF configuration
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