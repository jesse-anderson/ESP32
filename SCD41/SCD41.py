# SCD41.py
#This code is pretty heavily based off of the Sensirion type code at: https://github.com/octaprog7/SCD4x
# And this library written by Sensirion: https://github.com/Sensirion/python-i2c-scd
# This file should be saved as SCD41.py and called as import SCD41. See main.py in this folder for an example usage.
import time
import math
from machine import SoftI2C, Pin, SPI
import micropython
import ustruct

@micropython.native
def _calc_crc(sequence) -> int:
    """
    Calculate CRC-8 checksum for the given sequence.
    """
    return crc8(sequence, polynomial=0x31, init_value=0xFF)

@micropython.native
def check_value(value: int, valid_range, error_msg: str) -> int:
    """
    Check if the value is within the valid range. Raise ValueError if not.
    """
    if value not in valid_range:
        raise ValueError(error_msg)
    return value

class Device:
    """Base class for devices."""
    def __init__(self, adapter, address: [int, SPI], big_byte_order: bool):
        """
        Initialize the device with adapter, address, and byte order.
        """
        self.adapter = adapter
        self.address = address
        self.big_byte_order = big_byte_order
        self.msb_first = True

    def _get_byteorder_as_str(self) -> tuple:
        """
        Return the byte order as a string ('big' or 'little') and format character ('>' or '<').
        """
        if self.is_big_byteorder():
            return 'big', '>'
        else:
            return 'little', '<'

    def unpack(self, fmt_char: str, source: bytes, redefine_byte_order: str = None) -> tuple:
        """
        Unpack the given source bytes according to the format character and byte order.
        """
        if not fmt_char:
            raise ValueError(f"Invalid length fmt_char parameter: {len(fmt_char)}")
        bo = self._get_byteorder_as_str()[1]
        if redefine_byte_order is not None:
            bo = redefine_byte_order[0]
        return ustruct.unpack(bo + fmt_char, source)

    @micropython.native
    def is_big_byteorder(self) -> bool:
        """
        Check if the device uses big byte order.
        """
        return self.big_byte_order

class BaseSensor(Device):
    """Base class for sensors."""
    def get_id(self):
        raise NotImplementedError

    def soft_reset(self):
        raise NotImplementedError

class Iterator:
    """Iterator class for sensors."""
    def __iter__(self):
        return self

    def __next__(self):
        raise NotImplementedError

class BitField:
    """Class for working with bit fields."""
    def __init__(self, start: int, stop: int, alias: [str, None]):
        """
        Initialize the bit field with start, stop, and alias.
        """
        check(start, stop)
        self.alias = alias
        self.start = start
        self.stop = stop
        self.bitmask = _bitmask(start, stop)

    def put(self, source: int, value: int) -> int:
        """
        Write the value to the specified bit range in the source.
        """
        src = source & ~self.bitmask
        src |= (value << self.start) & self.bitmask
        return src

    def get(self, source: int) -> int:
        """
        Get the value from the specified bit range in the source.
        """
        return (source & self.bitmask) >> self.start

@micropython.native
def put(start: int, stop: int, source: int, value: int) -> int:
    """
    Write the value to the specified bit range in the source.
    """
    check(start, stop)
    bitmask = _bitmask(start, stop)
    src = source & bitmask
    src |= (value << start) & bitmask
    return src

@micropython.native
def _bitmask(start: int, stop: int) -> int:
    """
    Generate a bitmask from start to stop bits.
    """
    res = 0
    for i in range(start, 1 + stop):
        res |= 1 << i
    return res

def check(start: int, stop: int):
    """
    Check if start is less than or equal to stop. Raise ValueError if not.
    """
    if start > stop:
        raise ValueError(f"Invalid start: {start}, stop value: {stop}")

def _mpy_bl(value: int) -> int:
    """
    Calculate the bit length of the value.
    """
    if 0 == value:
        return 0
    return 1 + int(math.log2(abs(value)))

class BusAdapter:
    """Adapter class for bus communication."""
    def __init__(self, bus: [I2C, SPI]):
        """
        Initialize the adapter with the bus.
        """
        self.bus = bus

    def get_bus_type(self) -> type:
        """
        Return the type of the bus.
        """
        return type(self.bus)

    def read_register(self, device_addr: [int, Pin], reg_addr: int, bytes_count: int) -> bytes:
        raise NotImplementedError

    def write_register(self, device_addr: [int, Pin], reg_addr: int, value: [int, bytes, bytearray],
                       bytes_count: int, byte_order: str):
        raise NotImplementedError

    def read(self, device_addr: [int, Pin], n_bytes: int) -> bytes:
        raise NotImplementedError

    def write(self, device_addr: [int, Pin], buf: bytes):
        raise NotImplementedError

    def write_const(self, device_addr: [int, Pin], val: int, count: int):
        """
        Write a constant value to the device multiple times.
        """
        if 0 == count:
            return
        bl = _mpy_bl(val)
        if bl > 8:
            raise ValueError(f"The value must take no more than 8 bits! Current: {bl}")
        _max = 16
        if count < _max:
            _max = count
        repeats = count // _max
        b = bytearray([val for _ in range(_max)])
        for _ in range(repeats):
            self.write(device_addr, b)
        remainder = count - _max * repeats
        if remainder:
            b = bytearray([val for _ in range(remainder)])
            self.write(device_addr, b)

class I2cAdapter(BusAdapter):
    """Adapter class for I2C bus communication."""
    def __init__(self, bus: I2C):
        super().__init__(bus)

    def write_register(self, device_addr: int, reg_addr: int, value: [int, bytes, bytearray],
                       bytes_count: int, byte_order: str):
        """
        Write to the register of the device.
        """
        buf = None
        if isinstance(value, int):
            buf = value.to_bytes(bytes_count, byte_order)
        if isinstance(value, (bytes, bytearray)):
            buf = value
        return self.bus.writeto_mem(device_addr, reg_addr, buf)

    def read_register(self, device_addr: int, reg_addr: int, bytes_count: int) -> bytes:
        """
        Read from the register of the device.
        """
        return self.bus.readfrom_mem(device_addr, reg_addr, bytes_count)

    def read(self, device_addr: int, n_bytes: int) -> bytes:
        """
        Read bytes from the device.
        """
        return self.bus.readfrom(device_addr, n_bytes)

    def readfrom_into(self, device_addr: int, buf):
        """
        Read bytes from the device into the buffer.
        """
        return self.bus.readfrom_into(device_addr, buf)

    def read_buf_from_mem(self, device_addr: int, mem_addr, buf):
        """
        Read bytes from the device memory into the buffer.
        """
        return self.bus.readfrom_mem_into(device_addr, mem_addr, buf)

    def write(self, device_addr: int, buf: bytes):
        """
        Write bytes to the device.
        """
        return self.bus.writeto(device_addr, buf)

    def write_buf_to_mem(self, device_addr: int, mem_addr, buf):
        """
        Write bytes to the device memory.
        """
        return self.bus.writeto_mem(device_addr, mem_addr, buf)

class SCD4xSensirion(BaseSensor, Iterator):
    """Class for SCD4x Sensirion CO2 sensor."""
    def __init__(self, adapter: I2cAdapter, address=0x62, this_is_scd41: bool = True, check_crc: bool = True):
        """
        Initialize the sensor with adapter, address, and settings.
        """
        super().__init__(adapter, address, True)
        self._buf_3 = bytearray((0 for _ in range(3)))
        self._buf_9 = bytearray((0 for _ in range(9)))
        self.check_crc = check_crc
        self._low_power_mode = False
        self._single_shot_mode = False
        self._rht_only = False
        self._isSCD41 = this_is_scd41
        self.byte_order = self._get_byteorder_as_str()

    def _get_local_buf(self, bytes_for_read: int) -> [None, bytearray]:
        """
        Return the local buffer for reading.
        """
        if bytes_for_read not in (0, 3, 9):
            raise ValueError(f"Invalid value for bytes_for_read: {bytes_for_read}")
        if not bytes_for_read:
            return None
        if 3 == bytes_for_read:
            return self._buf_3
        return self._buf_9

    def _to_bytes(self, value, length: int):
        """
        Convert value to bytes with specified length.
        """
        byteorder = self.byte_order[0]
        return value.to_bytes(length, byteorder)

    def _write(self, buf: bytes) -> bytes:
        """
        Write buffer to the device.
        """
        return self.adapter.write(self.address, buf)

    def _readfrom_into(self, buf):
        """
        Read bytes from the device into the buffer.
        """
        return self.adapter.readfrom_into(self.address, buf)

    def _send_command(self, cmd: int, value: [bytes, None], wait_time: int = 0, bytes_for_read: int = 0,
                      crc_index: range = None, value_index: tuple = None) -> [bytes, None]:
        """
        Send a command to the sensor.
        """
        raw_cmd = self._to_bytes(cmd, 2)
        raw_out = raw_cmd
        if value:
            raw_out += value
            raw_out += self._to_bytes(_calc_crc(value), 1)
        self._write(raw_out)
        if wait_time:
            time.sleep_ms(wait_time)
        if not bytes_for_read:
            return None
        b = self._get_local_buf(bytes_for_read)
        self._readfrom_into(b)
        check_value(len(b), (bytes_for_read,), f"Invalid buffer length for cmd: {cmd}. Received {len(b)} out of {bytes_for_read}")
        if self.check_crc:
            crc_from_buf = [b[i] for i in crc_index]
            calculated_crc = [_calc_crc(b[rng.start:rng.stop]) for rng in value_index]
            if crc_from_buf != calculated_crc:
                raise ValueError(f"Invalid CRC! Calculated{calculated_crc}. From buffer {crc_from_buf}")
        return b

    def save_config(self):
        """
        Save the sensor configuration to EEPROM.
        """
        cmd = 0x3615
        self._send_command(cmd, None, 800)

    def get_id(self) -> tuple:
        """
        Get the unique serial number of the sensor.
        """
        cmd = 0x3682
        b = self._send_command(cmd, None, 0, bytes_for_read=9,
                               crc_index=range(2, 9, 3), value_index=(range(2), range(3, 5), range(6, 8)))
        return tuple([(b[i] << 8) | b[i+1] for i in range(0, 9, 3)])

    def soft_reset(self):
        """
        Perform a soft reset of the sensor.
        """
        return None

    def exec_self_test(self) -> bool:
        """
        Execute self-test on the sensor. Returns True if successful.
        """
        cmd = 0x3639
        length = 3
        b = self._send_command(cmd, None, wait_time=10_000,
                               bytes_for_read=length, crc_index=range(2, 3), value_index=(range(2),))
        res = self.unpack("H", b)[0]
        return 0 == res

    def reinit(self) -> None:
        """
        Reinitialize the sensor by reloading user settings from EEPROM.
        """
        cmd = 0x3646
        self._send_command(cmd, None, 20)

    def set_temperature_offset(self, offset: float):
        """
        Set the temperature offset for the sensor.
        """
        cmd = 0x241D
        offset_raw = self._to_bytes(int(374.49142857 * offset), 2)
        self._send_command(cmd, offset_raw, 1)

    def get_temperature_offset(self) -> float:
        """
        Get the temperature offset from the sensor.
        """
        cmd = 0x2318
        b = self._send_command(cmd, None, wait_time=1, bytes_for_read=3, crc_index=range(2, 3), value_index=(range(2),))
        temp_offs = self.unpack("H", b)[0]
        return 0.0026702880859375 * temp_offs

    def set_altitude(self, masl: int):
        """
        Set the altitude for the sensor in meters above sea level.
        """
        cmd = 0x2427
        masl_raw = self._to_bytes(masl, 2)
        self._send_command(cmd, masl_raw, 1)

    def get_altitude(self) -> int:
        """
        Get the altitude from the sensor in meters above sea level.
        """
        cmd = 0x2322
        b = self._send_command(cmd, None, wait_time=1, bytes_for_read=3, crc_index=range(2, 3), value_index=(range(2),))
        return self.unpack("H", b)[0]

    def set_ambient_pressure(self, pressure: float):
        """
        Set the ambient pressure for the sensor in Pascals.
        """
        cmd = 0xE000
        press_raw = self._to_bytes(int(pressure // 100), 2)
        self._send_command(cmd, press_raw, 1)

    def force_recalibration(self, target_co2_concentration: int) -> int:
        """
        Force recalibration of the sensor with the target CO2 concentration.
        Returns the recalibration correction value and prints detailed debug information.
        """
        # Check if the target CO2 concentration is within the valid range
        check_value(target_co2_concentration, range(2**16),
                    f"Invalid target CO2 concentration: {target_co2_concentration} ppm")
        
        # Command for forced recalibration
        cmd = 0x362F
        
        # Convert the target CO2 concentration to bytes
        target_raw = self._to_bytes(target_co2_concentration, 2)
        
        # Calculate CRC for the target concentration
        target_crc = self._to_bytes(_calc_crc(target_raw), 1)
        
        # Combine the target concentration and its CRC into the payload
        payload = target_raw + target_crc
        
        # Print the command, target concentration, and CRC for debugging
        print(f"Performing forced recalibration with target CO2 concentration: {target_co2_concentration} ppm")
        print(f"Write {cmd:#04x} {target_raw.hex()} {target_crc.hex()}")
        
        # Send the command to the sensor and wait for the response
        b = self._send_command(cmd, payload, 400, 3, crc_index=range(2, 3), value_index=(range(2),))
        
        # Unpack the response to get the recalibration correction value
        correction_value = self.unpack("h", b)[0]
        
        # Print the response for debugging
        print(f"Response: {b.hex()} Correction value: {correction_value} ppm")
        
        # Determine the status message based on the correction value
        if correction_value == 0xFFFF:
            status_message = "Forced recalibration failed."
        elif correction_value == 0:
            status_message = "No correction needed, sensor is already calibrated."
        else:
            status_message = f"Forced recalibration completed with correction value: {correction_value} ppm."

        # Print the status message
        print(status_message)
        
        # Return the recalibration correction value
        return correction_value



    def is_auto_calibration(self) -> bool:
        """
        Check if automatic self-calibration is enabled on the sensor.
        """
        cmd = 0x2313
        b = self._send_command(cmd, None, 1, 3, crc_index=range(2, 3), value_index=(range(2),))
        return 0 != self.unpack("H", b)[0]

    def set_auto_calibration(self, value: bool):
        """
        Enable or disable automatic self-calibration on the sensor.
        """
        cmd = 0x2416
        value_raw = self._to_bytes(1 if value else 0, 2)  # Convert bool to int before converting to bytes
        value_crc = self._to_bytes(_calc_crc(value_raw), 1)
        payload = value_raw + value_crc
        self._send_command(cmd, payload, 1)

    def set_measurement(self, start: bool, single_shot: bool = False, rht_only: bool = False):
        """
        Start or stop periodic measurements, or perform a single shot measurement.
        """
        if single_shot:
            return self._single_shot_meas(rht_only)
        return self._periodic_measurement(start)

    def _periodic_measurement(self, start: bool):
        """
        Start or stop periodic measurements.
        """
        wt = 0
        if start:
            cmd = 0x21AC if self._low_power_mode else 0x21B1
        else:
            cmd = 0x3F86
            wt = 500
        self._send_command(cmd, None, wt)
        self._single_shot_mode = False
        self._rht_only = False

    def get_meas_data(self) -> tuple:
        """
        Get the measurement data from the sensor (CO2, temperature, and humidity).
        """
        cmd = 0xEC05
        val_index = (range(2), range(3, 5), range(6, 8))
        b = self._send_command(cmd, None, 1, bytes_for_read=9,
                               crc_index=range(2, 9, 3), value_index=val_index)
        words = [self.unpack("H", b[val_rng.start:val_rng.stop])[0] for val_rng in val_index]
        return words[0], -45 + 0.0026703288 * words[1], 0.0015259022 * words[2]

    def is_data_ready(self) -> bool:
        """
        Check if the measurement data is ready to be read from the sensor.
        """
        cmd = 0xE4B8
        b = self._send_command(cmd, None, 1, 3, crc_index=range(2, 3), value_index=(range(2),))
        return bool(self.unpack("H", b)[0] & 0b0000_0111_1111_1111)

    @micropython.native
    def get_conversion_cycle_time(self) -> int:
        """
        Get the conversion cycle time of the sensor in milliseconds.
        """
        if self._single_shot_mode and self._rht_only:
            return 50  # RHT only single shot mode conversion time
        elif self._single_shot_mode:
            return 5000  # CO2 + RHT single shot mode conversion time
        elif self._low_power_mode:
            return 30000  # Low power periodic mode conversion time
        else:
            return 5000  # Default periodic mode conversion time

    def set_power(self, value: bool):
        """
        Power up or power down the sensor.
        """
        if not self._isSCD41:
            return
        cmd = 0x36F6 if value else 0x36E0
        wt = 20 if value else 1
        self._send_command(cmd, None, wt)

    def _single_shot_meas(self, rht_only: bool = False):
        """
        Perform a single shot measurement.
        """
        if not self._isSCD41:
            return
        cmd = 0x2196 if rht_only else 0x219D
        self._send_command(cmd, None, 0)
        self._single_shot_mode = True
        self._rht_only = rht_only

    @property
    def is_single_shot_mode(self) -> bool:
        """
        Check if the sensor is in single shot mode.
        """
        return self._single_shot_mode

    @property
    def is_rht_only(self) -> bool:
        """
        Check if the sensor is in RHT-only mode.
        """
        return self._rht_only
    
    def perform_factory_reset(self):
        """
        Perform a factory reset of the sensor, resetting all configuration settings stored in the EEPROM
        and erasing the FRC and ASC algorithm history.
        """
        cmd = 0x3632
        self._send_command(cmd, None, wait_time=1200)
    
    def __iter__(self):
        return self

    def __next__(self) -> [tuple, None]:
        """
        Get the next set of measurement data.
        """
        if self._single_shot_mode:
            return None
        if self.is_data_ready():
            return self.get_meas_data()
        return None

def pa_mmhg(value: float) -> float:
    """
    Convert air pressure from Pascals to millimeters of mercury.
    """
    return 7.50062E-3 * value

def crc8(sequence, polynomial: int, init_value: int = 0x00):
    """
    Calculate CRC-8 checksum for the given sequence.
    """
    mask = 0xFF
    crc = init_value & mask
    for item in sequence:
        crc ^= item & mask
        for _ in range(8):
            if crc & 0x80:
                crc = mask & ((crc << 1) ^ polynomial)
            else:
                crc = mask & (crc << 1)
    return crc

def check_device_presence(i2c, address):
    """
    Check if a device with the given address is present on the I2C bus.
    """
    devices = i2c.scan()
    return address in devices
