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

class ToFManager:
    def __init__(self, i2c):
        self.i2c = i2c
        self.count = 8
        self.addresses = config.tof_addresses
        self.enabled = {addr: config.tof_enabled[addr] for addr in config.tof_addresses}
        self.tofs = [SteelBarToF(i2c, addr, config.tof_angles[addr], config.tof_offsets[addr]) for addr in config.tof_addresses]
        self.distances = {addr: 0 for addr in config.tof_addresses}
        self.offsets = {addr: config.tof_offsets[addr] for addr in config.tof_addresses}
        self.angles = {addr: config.tof_angles[addr] for addr in config.tof_addresses}
    
    def _read_one(self, addr, wait_for_new):
        out_buf = bytes([READ_REG])
        in_buf = bytearray(5)
        while True:
            self.i2c.writeto_then_readfrom(addr, out_buf, in_buf)
            seq = in_buf[0]
            distance_mm = struct.unpack('<i', in_buf[1:5])[0] 
            if (not wait_for_new) or (self.last_sequences[addr] is None) or (seq != self.last_sequences[addr]):
                self.last_sequences[addr] = seq
                if distance_mm > 0:
                    self.distances[addr] = distance_mm
                    distance_mm += self.offsets[addr]
                return self.distances[addr], seq
    
    def _batch_read(self, wait_for_new):
        results = {}
        for addr in self.addresses:
            if not self.enabled.get(addr, True):
                continue
            try:
                d, seq = self._read_one(addr, wait_for_new)
                results[addr] = ToFReading(
                    address=addr,
                    distance_mm=d,
                    angle_rad=self.angles.get(addr),
                )
            except Exception as e:
                # who wants error handling anyways
                pass
        return results
    
    def batch_current(self):
        return self._batch_read(wait_for_new=False)

    def batch_next(self):
        return self._batch_read(wait_for_new=True)

    

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
                    last_distance = distance_mm
                    return distance_mm
                else:
                    return last_distance
            
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
        mgr = ToFManager(i2c)
    finally:
        i2c.unlock()