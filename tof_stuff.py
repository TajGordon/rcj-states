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

# Manager for reading all ToFs and returning distance from robot center with angle
class ToFArray:
    def __init__(self, i2c):
        self.i2c = i2c
        if not hasattr(config, 'tof_addresses'):
            raise RuntimeError('config.tof_addresses not defined. Add tof_* settings in config.py')
        self.sensors = []
        for addr in config.tof_addresses:
            if getattr(config, 'tof_enabled', {}).get(addr, True):
                angle = getattr(config, 'tof_angles', {}).get(addr)
                offset = getattr(config, 'tof_offsets', {}).get(addr, 0)
                self.sensors.append(SteelBarToF(i2c, addr, angle, offset))

    def read_all_current(self):
        """Return list[ToFReading] using current readings (may repeat last). distance_mm is from center."""
        results = []
        for s in self.sensors:
            try:
                d = s.current_measurement()
                if d > 0:
                    distance_from_center = int(max(0, d + (s.offset or 0)))
                    results.append(ToFReading(address=s.address, distance_mm=distance_from_center, angle_rad=s.angle))
            except Exception:
                pass
        return results

    def read_all_next(self):
        """Return list[ToFReading] waiting for next fresh sample. distance_mm is from center."""
        results = []
        for s in self.sensors:
            try:
                d = s.next_measurement()
                if d > 0:
                    distance_from_center = int(max(0, d + (s.offset or 0)))
                    results.append(ToFReading(address=s.address, distance_mm=distance_from_center, angle_rad=s.angle))
            except Exception:
                pass
        return results

    def get_localization_pairs(self, fresh=False):
        """Return list of (angle_rad, distance_from_center_mm) for localization."""
        readings = self.read_all_next() if fresh else self.read_all_current()
        return [(r.angle_rad, r.distance_mm) for r in readings if r.angle_rad is not None]

# test code
if __name__ == "__main__":
    print('testing tof sensors')
    i2c = busio.I2C(board.SCL, board.SDA)
    while not i2c.try_lock():
        pass
    mode = input('what mode would you like to test? (single|multi)')
    # tof_count = int(input('how many tofs do you want: '))
    try:
        if mode == 'single':
            addr = input('what is the address of the tof?: ')
            tof = SteelBarToF(i2c, addr, 0, 41.5)
            while True:
                d = tof.next_measurement()
                print(f"{d} mm")
                time.sleep(0.2)
        elif mode == 'multi':
            array = ToFArray(i2c)
            while True:
                pairs = array.get_localization_pairs(fresh=True)
                print(pairs)
                time.sleep(0.2)
        else:
            print('invalid mode')
    finally:
        i2c.unlock()