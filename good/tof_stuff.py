from asyncio import wait_for
from dataclasses import dataclass
import time
import struct
from typing import Optional, List
import board
import busio
import config
import statistics

READ_REG = 0x10 # address to write to to request data apparently

@dataclass
class ToFReading:
    address: int
    distance_mm: int
    angle_rad: Optional[float]
    is_valid: bool = True  # Whether this reading passed filtering

class SteelBarToF:
    def __init__(self, i2c, address, angle, offset):
        self.i2c = i2c
        self.address = address
        self.angle = angle
        self.offset = offset
        self.last_distance = 0
        self.last_sequence = None
        
        # Filtering parameters
        self.min_distance = 20  # Minimum valid distance (mm)
        self.max_distance = 3000  # Maximum valid distance (mm)
        self.max_change_rate = 500  # Maximum change per reading (mm)
        self.reading_history = []  # History for median filtering
        self.history_size = 5  # Number of readings to keep for median filter
        self.consecutive_invalid = 0  # Count of consecutive invalid readings
        self.max_consecutive_invalid = 3  # Max invalid readings before using last valid
        self.last_valid_distance = 0  # Last known good reading
    
    def _filter_reading(self, raw_distance):
        """Apply filtering to a raw distance reading"""
        # Check for obviously invalid readings
        if raw_distance <= 0 or raw_distance < self.min_distance or raw_distance > self.max_distance:
            self.consecutive_invalid += 1
            if self.consecutive_invalid <= self.max_consecutive_invalid and self.last_valid_distance > 0:
                # Use last valid reading if we haven't had too many consecutive invalid readings
                return self.last_valid_distance, False
            else:
                # Too many invalid readings, return 0 (no obstacle detected)
                return 0, False
        
        # Check for sudden changes (outlier detection)
        if self.last_valid_distance > 0:
            change = abs(raw_distance - self.last_valid_distance)
            if change > self.max_change_rate:
                self.consecutive_invalid += 1
                if self.consecutive_invalid <= self.max_consecutive_invalid:
                    # Use last valid reading for sudden changes
                    return self.last_valid_distance, False
                else:
                    # Too many outliers, accept the reading but mark as potentially invalid
                    self.consecutive_invalid = 0
                    self.last_valid_distance = raw_distance
                    return raw_distance, True
        
        # Reading passed basic validation
        self.consecutive_invalid = 0
        self.last_valid_distance = raw_distance
        
        # Add to history for median filtering
        self.reading_history.append(raw_distance)
        if len(self.reading_history) > self.history_size:
            self.reading_history.pop(0)
        
        # Apply median filter if we have enough history
        if len(self.reading_history) >= 3:
            filtered_distance = statistics.median(self.reading_history)
            return int(filtered_distance), True
        else:
            return raw_distance, True

    def _read(self, wait_for_new):
        write_buf = bytes([READ_REG])
        read_buf = bytearray(5)

        while True:
            self.i2c.writeto(self.address, write_buf)
            self.i2c.readfrom_into(self.address, read_buf)
            seq = read_buf[0]
            raw_distance = struct.unpack('<i', read_buf[1:5])[0]
            
            if (not wait_for_new) or (self.last_sequence is None) or (seq != self.last_sequence):
                self.last_sequence = seq
                
                # Apply filtering
                filtered_distance, is_valid = self._filter_reading(raw_distance)
                self.last_distance = filtered_distance
                return self.last_distance, is_valid
            
            time.sleep(0.000001)
    
    def current_measurement(self):
        distance, is_valid = self._read(wait_for_new=False)
        return distance
    
    def next_measurement(self):
        distance, is_valid = self._read(wait_for_new=True)
        return distance
    
    def current_measurement_with_validity(self):
        """Return (distance, is_valid) tuple"""
        return self._read(wait_for_new=False)
    
    def next_measurement_with_validity(self):
        """Return (distance, is_valid) tuple"""
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
                d, is_valid = s.current_measurement_with_validity()
                if d > 0:
                    distance_from_center = int(max(0, d + (s.offset or 0)))
                    results.append(ToFReading(address=s.address, distance_mm=distance_from_center, angle_rad=s.angle, is_valid=is_valid))
            except Exception:
                pass
        return results

    def read_all_next(self):
        """Return list[ToFReading] waiting for next fresh sample. distance_mm is from center."""
        results = []
        for s in self.sensors:
            try:
                d, is_valid = s.next_measurement_with_validity()
                if d > 0:
                    distance_from_center = int(max(0, d + (s.offset or 0)))
                    results.append(ToFReading(address=s.address, distance_mm=distance_from_center, angle_rad=s.angle, is_valid=is_valid))
            except Exception:
                pass
        return results

    def get_localization_pairs(self, fresh=False, valid_only=True):
        """Return list of (angle_rad, distance_from_center_mm) for localization."""
        readings = self.read_all_next() if fresh else self.read_all_current()
        if valid_only:
            return [(r.angle_rad, r.distance_mm) for r in readings if r.angle_rad is not None and r.is_valid]
        else:
            return [(r.angle_rad, r.distance_mm) for r in readings if r.angle_rad is not None]
    
    def get_valid_readings(self, fresh=False):
        """Return only readings that passed filtering."""
        readings = self.read_all_next() if fresh else self.read_all_current()
        return [r for r in readings if r.is_valid]
    
    def get_filter_stats(self):
        """Return statistics about filtering for each sensor."""
        stats = []
        for s in self.sensors:
            stats.append({
                'address': s.address,
                'consecutive_invalid': s.consecutive_invalid,
                'last_valid_distance': s.last_valid_distance,
                'history_size': len(s.reading_history)
            })
        return stats
    
    def reset_filters(self):
        """Reset all filtering state for all sensors."""
        for s in self.sensors:
            s.consecutive_invalid = 0
            s.reading_history = []
            s.last_valid_distance = 0

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
            addr = int(input('what is the address of the tof?: '))
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