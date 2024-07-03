import ustruct
import time

class TCS34725:
    _COMMAND_BIT = 0x80
    _REGISTER_ENABLE = 0x00
    _ENABLE_AIEN = 0x10
    _ENABLE_WEN = 0x08
    _ENABLE_AEN = 0x02
    _ENABLE_PON = 0x01
    _REGISTER_CONTROL = 0x0F
    _REGISTER_SENSORID = 0x12
    _REGISTER_STATUS = 0x13
    _REGISTER_CDATA = 0x14
    _REGISTER_RDATA = 0x16
    _REGISTER_GDATA = 0x18
    _REGISTER_BDATA = 0x1A

    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self.address = address
        self._active = False
        self.integration_time(2.4)
        sensor_id = self.sensor_id()
        print("Detected sensor ID:", hex(sensor_id))

    def _register8(self, register, value=None):
        register |= self._COMMAND_BIT
        if value is None:
            return self.i2c.readfrom_mem(self.address, register, 1)[0]
        data = ustruct.pack('<B', value)
        self.i2c.writeto_mem(self.address, register, data)

    def _register16(self, register, value=None):
        register |= self._COMMAND_BIT
        if value is None:
            data = self.i2c.readfrom_mem(self.address, register, 2)
            return ustruct.unpack('<H', data)[0]
        data = ustruct.pack('<H', value)
        self.i2c.writeto_mem(self.address, register, data)

    def active(self, value=None):
        if value is None:
            return self._active
        value = bool(value)
        if self._active == value:
            return
        self._active = value
        enable = self._register8(self._REGISTER_ENABLE)
        if value:
            self._register8(self._REGISTER_ENABLE, enable | self._ENABLE_PON)
            time.sleep_ms(3)
            self._register8(self._REGISTER_ENABLE, enable | self._ENABLE_PON | self._ENABLE_AEN)
        else:
            self._register8(self._REGISTER_ENABLE, enable & ~(self._ENABLE_PON | self._ENABLE_AEN))

    def sensor_id(self):
        return self._register8(self._REGISTER_SENSORID)

    def integration_time(self, value=None):
        if value is None:
            return self._integration_time
        value = min(614.4, max(2.4, value))
        cycles = int(value / 2.4)
        self._integration_time = cycles * 2.4
        return self._register8(0x01, 256 - cycles)

    def gain(self, value=None):
        _GAINS = (1, 4, 16, 60)
        if value is None:
            return _GAINS[self._register8(self._REGISTER_CONTROL)]
        if value not in _GAINS:
            raise ValueError("gain must be 1, 4, 16 or 60")
        return self._register8(self._REGISTER_CONTROL, _GAINS.index(value))

    def _valid(self):
        return bool(self._register8(self._REGISTER_STATUS) & 0x01)

    def read(self, raw=False):
        was_active = self.active()
        self.active(True)
        timeout = time.ticks_ms() + 5000  # 1 second timeout
        while not self._valid():
            if time.ticks_ms() > timeout:
                raise RuntimeError("Timeout waiting for TCS34725 sensor data")
            time.sleep_ms(int(self._integration_time + 0.9))
        data = tuple(self._register16(register) for register in (
            self._REGISTER_RDATA,
            self._REGISTER_GDATA,
            self._REGISTER_BDATA,
            self._REGISTER_CDATA,
        ))
        self.active(was_active)
        if raw:
            return data
        return self._temperature_and_lux(data)

    def _temperature_and_lux(self, data):
        r, g, b, c = data
        if c == 0:  # Prevent divide by zero
            cct = 0
            lux = 0
        else:
            x = -0.14282 * r + 1.54924 * g + -0.95641 * b
            y = -0.32466 * r + 1.57837 * g + -0.73191 * b
            z = -0.68202 * r + 0.77073 * g +  0.56332 * b
            d = x + y + z
            if d == 0:
                cct = 0
                lux = y  # Lux is proportional to y
            else:
                n = (x / d - 0.3320) / (0.1858 - y / d)
                cct = 449.0 * n**3 + 3525.0 * n**2 + 6823.3 * n + 5520.33
                lux = y
        return cct, lux

    def led(self, state):
        """Enable or disable the LED."""
        enable = self._register8(self._REGISTER_ENABLE)
        if state:
            enable |= self._ENABLE_WEN
        else:
            enable &= ~self._ENABLE_WEN
        self._register8(self._REGISTER_ENABLE, enable)
        print(f"LED {'enabled' if state else 'disabled'}")

    def threshold(self, cycles=None, min_value=None, max_value=None):
        if cycles is None and min_value is None and max_value is None:
            min_value = self._register16(0x04)
            max_value = self._register16(0x06)
            if self._register8(0x00) & self._ENABLE_AIEN:
                cycles = (0, 1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60)[self._register8(0x0c) & 0x0f]
            else:
                cycles = -1
            return cycles, min_value, max_value
        if min_value is not None:
            self._register16(0x04, min_value)
        if max_value is not None:
            self._register16(0x06, max_value)
        if cycles is not None:
            enable = self._register8(0x00)
            if cycles == -1:
                self._register8(0x00, enable & ~self._ENABLE_AIEN)
            else:
                self._register8(0x00, enable | self._ENABLE_AIEN)
                if cycles not in (0, 1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60):
                    raise ValueError("invalid persistence cycles")
                self._register8(0x0c, (0, 1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60).index(cycles))

    def interrupt(self, value=None):
        if value is None:
            return bool(self._register8(0x13) & self._ENABLE_AIEN)
        if value:
            raise ValueError("interrupt can only be cleared")
        self.i2c.writeto(self.address, b'\xe6')

    def html_rgb(self, data):
        r, g, b, c = data
        if c == 0:
            return 0, 0, 0
        red = (r / c) * 255
        green = (g / c) * 255
        blue = (b / c) * 255
        return red, green, blue

    def html_hex(self, data):
        r, g, b = self.html_rgb(data)
        return "{0:02x}{1:02x}{2:02x}".format(int(r), int(g), int(b))

# def html_rgb(data):
#     r, g, b, c = data
#     if c == 0:
#         return 0, 0, 0
#     red = pow((int((r/c) * 256) / 255), 2.5) * 255# we can get 16 bit RGB values here, but I want 8 bit 0-255 range!
#     green = pow((int((g/c) * 256) / 255), 2.5) * 255
#     blue = pow((int((b/c) * 256) / 255), 2.5) * 255
#     return red, green, blue