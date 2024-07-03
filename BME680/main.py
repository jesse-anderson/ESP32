# Example usage for ESP32
import machine
import time
from BME680 import BME680_I2C

i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
sensor = BME680_I2C(i2c)

while True:
    temperature = sensor.temperature
    pressure = sensor.pressure
    humidity = sensor.humidity
    gas = sensor.gas
    altitude = sensor.altitude

    print("Temperature: {:.2f} C".format(temperature))
    print("Pressure: {:.2f} hPa".format(pressure))
    print("Humidity: {:.2f} %".format(humidity))
    print("Gas: {} ohms".format(gas))
    print("Altitude: {:.2f} m".format(altitude))
    print("")
    time.sleep(1)