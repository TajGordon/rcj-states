from asyncio import wait_for
from dataclasses import dataclass
import time
import struct
from typing import Optional
import board
import busio
import config

READ_REG = 0x10 # address to write to to request data apparently

@dataclass
class ToFReading:
    address: int
    distance_mm: int
    angle_rad: Optional[float]

class SteelBarToF:
    def __init__(self, i2c, address, angle, offset):
        self.i2c = i2c
        self.address = address
        self.angle = angle
        self.offset = offset
        self.last_distance = 0
        self.last_sequence = None
    
    def _read(self, wait_for_new):
        write_buf = bytes([READ_REG])
        read_buf = bytearray(5)

        while True:
            self.i2c.writeto(self.address, write_buf)
            self.i2c.readfrom_into(self.address, read_buf)
            seq = read_buf[0]
            distance_mm = struct.unpack('<i', read_buf[1:5])[0]
            if (not wait_for_new) or (self.last_sequence is None) or (seq != self.last_sequence):
                self.last_sequence = seq
                if distance_mm > 0:
                    self.last_distance = distance_mm
                    return self.last_distance
                else:
                    return self.last_distance
            
            time.sleep(0.00001)
    
    def current_measurement(self):
        return self._read(wait_for_new=False)
    
    def next_measurement(self):
        return self._read(wait_for_new=True)

# test code
if __name__ == "__main__":
    print('testing tof sensors')
    # tof_count = int(input('how many tofs do you want: '))
    i2c = busio.I2C(board.SCL, board.SDA)
    while not i2c.try_lock():
        pass
    try:
        tof = SteelBarToF(i2c, 0x50, 0, 41.5)
        while True:
            d = tof.next_measurement()
            print(f"{d} mm")
            time.sleep(0.2)
    finally:
        i2c.unlock()