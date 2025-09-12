import socket

hostname = socket.gethostname()
# motor addresses is: [back left, back right, front left, front right]

if hostname == 'hyperion':
    print('running on taj\'s bot')
    # front left: 26, front right: 27, back right: 29, back left: 25
    motor_addresses = [25, 29, 26, 27]
    # idk if these get used
    SINCOSCENTRE = {25: 1260, 26: 1250, 27: 1219, 29: 1256}
    ELECANGLEOFFSET = {25: 1452253440, 26: 1167990784, 27: 1478825728, 29: 1547735552}
else: # assuming siddak bot rn
    motor_addresses = [27, 28, 30, 26]
    print(f'running on {hostname}')