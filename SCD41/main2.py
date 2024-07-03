# main.py

from machine import SoftI2C, Pin
import time
from SCD41 import SCD4xSensirion, I2cAdapter, check_device_presence
from ssd1306 import SSD1306_I2C  # Ensure you have the SSD1306 library
import urequests as requests
import network
import json
import utime
import usocket as socket
import ssl
import ntptime

# ThingSpeak settings
THINGSPEAK_API_KEY = ''
THINGSPEAK_URL = 'https://api.thingspeak.com/update'
THINGSPEAK_CHANNEL_ID = '2579212'
THINGSPEAK_BULK_UPDATE_URL = 'https://api.thingspeak.com/channels/'+str(THINGSPEAK_CHANNEL_ID)+'/bulk_update.json'
SEND_TO_THINGSPEAK = True

thingspeak_buffer = []  # Buffer for ThingSpeak data

# Google Sheets settings
SPREADSHEET_ID = ''
RANGE_NAME = 'Sheet1!A1:C1'
SHEET_NAME = 'Sheet1'
GOOGLE_URL = 'https://script.google.com/macros/s/xxxxxxx'
# WiFi settings
SSID = ''
PASSWORD = ''

switchOff = Pin(32, Pin.IN, Pin.PULL_UP)
switchOn = Pin(33, Pin.IN, Pin.PULL_UP)
switch_state = switchOn.value() # If switch is in up position we consider it on.
print(switch_state)
if switch_state == 0:
    print("Calibration switch is on! Calibrating Device...")
    
# Setup GPIO for OLED power
oled_power = Pin(19, Pin.OUT)

def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while not wlan.isconnected():
        time.sleep(1)
        print("Connecting to WiFi...")
    print("Connected to WiFi")
    print(wlan.ifconfig())
    
def get_time_chicago():
    max_retries = 100
    for attempt in range(max_retries):
        try:
            ntptime.settime()
            current_time = utime.localtime()
            break
        except OSError as e:
            print(f"Failed to get NTP time, attempt {attempt + 1} of {max_retries}. Error: {e}")
            time.sleep(1)
    else:
        print("Could not get NTP time, proceeding without time synchronization.")
        return utime.localtime()

    # Determine if it is daylight saving time (DST)
    month = current_time[1]
    day = current_time[2]
    hour = current_time[3]
    if (month > 3 and month < 11) or (month == 3 and day >= 8 and hour >= 2) or (month == 11 and day < 1 and hour < 2):
        is_dst = True
    else:
        is_dst = False
    
    offset = -6 * 3600 if not is_dst else -5 * 3600
    local_time = utime.mktime(current_time) + offset
    return utime.localtime(local_time)

def send_data_to_google_sheets(data):
    url = GOOGLE_URL  # Define your Google URL here
    headers = {'Content-Type': 'application/x-www-form-urlencoded'}
    encoded_data = (
        "Date=" + data['date'] +
        "&Time=" + data['time'] +
        "&CO2=" + str(data['co2']) +
        "&Temperature=" + str(data['temp_f']) +
        "&Humidity=" + str(data['humidity'])
    )
    try:
        # Extract host and path from URL
        _, _, host, path = url.split('/', 3)
        
        # Set up a socket connection
        addr = socket.getaddrinfo(host, 443)[0][-1]
        s = socket.socket()
        s.connect(addr)
        s = ssl.wrap_socket(s)
        
        # Create the HTTP request manually
        request = f"POST /{path} HTTP/1.1\r\nHost: {host}\r\n"
        request += "Content-Type: application/x-www-form-urlencoded\r\n"
        request += f"Content-Length: {len(encoded_data)}\r\n\r\n"
        request += encoded_data

        # Send the request
        s.write(request)
        
        # Close the socket
        s.close()
        print('Data sent to Google Sheets!')
    except Exception as e:
        print('Failed to send data to Google Sheets:', e)
        
def send_data_to_thingspeak():
    """Send data to ThingSpeak."""
    if SEND_TO_THINGSPEAK and thingspeak_buffer:
        if len(thingspeak_buffer) > 1:
            # Bulk update
            payload = {
                'write_api_key': THINGSPEAK_API_KEY,
                'updates': []
            }
            for data in thingspeak_buffer:
                update = {
                    'created_at': f"{data['date']} {data['time']} -0500",
                    'field1': data['co2'],
                    'field2': data['temp_f'],
                    'field3': data['humidity']
                }
                payload['updates'].append(update)

            try:
                headers = {'Content-Type': 'application/json'}
                json_data = json.dumps(payload)
                response = requests.post(THINGSPEAK_BULK_UPDATE_URL, headers=headers, data=json_data)
                if response.status_code == 202:
                    print('Data posted to ThingSpeak (bulk update):', response.text)
                    thingspeak_buffer.clear()  # Clear the buffer after successful update
                else:
                    print(f'Failed to send data to ThingSpeak (bulk update): {response.status_code}, {response.text}')
            except Exception as e:
                print('Failed to send data to ThingSpeak (bulk update):', e)
        else:
            data = thingspeak_buffer.pop(0)  # Get the first item in the buffer
            payload = {
                'api_key': THINGSPEAK_API_KEY,
                'field1': data['co2'],
                'field2': data['temp_f'],
                'field3': data['humidity']
            }
            try:
                response = requests.post(THINGSPEAK_URL, json=payload)
                if response.status_code == 200:
                    print('Data posted to ThingSpeak:', response.text)
                else:
                    print(f'Failed to send data to ThingSpeak: {response.status_code}, {response.text}')
            except Exception as e:
                print('Failed to send data to ThingSpeak:', e)

def get_sensor_reading(sensor, conversion_cycle_time,last_update_sensor):
    """Get a single sensor reading with a delay equal to the conversion cycle time."""
    update = False
    while update == False:
        print(time.time() - last_update_sensor)
        if time.time() - last_update_sensor >=30:       
            try:
#                 time.sleep_ms(conversion_cycle_time) #I want more fine grained timing of updating the sensor and sending the data so I don't run into power spikes...
                if sensor.is_data_ready():
                    update=True
                    return sensor.get_meas_data()
                else:
                    print("Data not ready.")
                    update=False
                    time.sleep(1)
                    return None
                    
            except OSError as e:
                print(f"Error during sensor reading: {e}")
                update=True
                return None
        else:
            time.sleep(1)
            update=False
led = Pin(2, Pin.OUT)
led.value(0)  # turn off red LED, doesn't work :L hard soldered in.

def main():
    # Connect to WiFi
    connect_wifi(SSID, PASSWORD)
    
    # Initialize I2C communication with the specified pins and frequency
    i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=400_000)
    device_address = 0x62

    # Check if the device is present on the I2C bus
    if not check_device_presence(i2c, device_address):
        print(f"Device with address {device_address} not found on I2C bus.")
        return

    print(f"Device with address {device_address} found on I2C bus.")
    oled_power.value(1)  # Turn on OLED display
    # Setup SoftI2C for OLED
    i2c_oled = SoftI2C(scl=Pin(4), sda=Pin(5))
    # Scan for OLED device
    print('Scanning for I2C devices...')
    devices = i2c_oled.scan()
    if len(devices) == 0:
        print("No I2C OLED devices found.")
        OLEDaddress = None
    else:
        print('I2C OLED devices found:', len(devices))
        OLEDaddress = devices[0]  # Assuming the first device found is the OLED
        for device in devices:
            print("Device address: ", hex(device))

    # Initialize OLED if found
    if OLEDaddress:
        print("Testing OLED...")
        oled = SSD1306_I2C(128, 64, i2c_oled)
        i = 0
        while i < 10:
            time.sleep(1)
            oled_power.value(1)
            oled.fill(0)
            oled.text(f"Init: Test {i}", 0, 0)
            oled.show()
            time.sleep(1)
            i = i+1
            oled_power.value(0)
            
    
    # Create an I2C adapter and sensor instance
    adapter = I2cAdapter(i2c)
    sensor = SCD4xSensirion(adapter)
    sensor.set_measurement(start=False, single_shot=False) #if looping need to put sensor back into IDLE mode. Sensor can't exec self test if currently reading...
    
    # Check if sensor is good to go. Note you may need to power cycle to pass this test. Make sure 5V in.
    selfTest = False
    while selfTest == False:
        try:
            if sensor.exec_self_test():
                print("Sensor self test shows good to go")
                selfTest = True
            else:
                print("Sensor self test failed!")
                selfTest = False
                time.sleep(1)
        except Exception as e:
            print("Exception occurred:", e)


    # Ensure the sensor is in IDLE mode
    sensor.set_measurement(start=False, single_shot=False)

    # Retrieve and display the sensor ID
    sensor_id = sensor.get_id()
    print(f"Sensor ID: {sensor_id[0]:x}:{sensor_id[1]:x}:{sensor_id[2]:x}")

    # Retrieve and display the temperature offset
    temp_offset = sensor.get_temperature_offset()
    print(f"Temperature offset: {temp_offset:.6f} °C")

    # Set and retrieve the altitude (meters above sea level)
    altitude = 198
    retrieved_altitude = sensor.get_altitude()
    print(f"Desired Altitude: {altitude} m, Retrieved Altitude: {retrieved_altitude} m")
    if altitude != retrieved_altitude:
        sensor.set_altitude(altitude)
        print(f"Altitude set to: {altitude}")
    
    # Check if automatic self-calibration is enabled
    if sensor.is_auto_calibration():
        print("Automatic self-calibration is ON.")
    else:
        print("Automatic self-calibration is OFF.")
    
    # Start periodic measurement
    sensor._low_power_mode = True
    sensor.set_measurement(start=True, single_shot=False)
    conversion_cycle_time = sensor.get_conversion_cycle_time()
    last_update_sensor =  time.time()
    print(f"Low Power Mode: {sensor._low_power_mode}")
    print(f"Low power periodic measurement started with conversion cycle time: {conversion_cycle_time} ms")
    print("Periodic measurement started.")

#     # Wait for two consecutive valid readings before entering the endless loop
#     valid_readings = 0
#     while valid_readings < 10:
#         reading = get_sensor_reading(sensor, conversion_cycle_time)
#         if reading:
#             co2, temp, humidity = reading
#             temp = temp * 9 / 5 +32
#             time.sleep(30)
#             print(f"Initial Reading {valid_readings + 1}: CO2: {co2} ppm, Temperature: {temp:.2f} °F, Humidity: {humidity:.2f} %")
#             valid_readings += 1
#         else:
#             # Reset the sensor and reinitialize if an error occurs
#             print("Attempting to reinitialize the sensor...")
#             sensor.set_measurement(start=False, single_shot=False)
#             sensor.set_measurement(start=True, single_shot=False)
#             valid_readings = 0
    if switch_state == 0:
        #### We wanna calibrate it with this chunk if anything...
        # Start periodic measurement
        print("Performing sensor calibration in 30 seconds...")
        print("The sensor will be calibrated over 60 min in fresh air.")
        time.sleep(30)
        sensorCycle = 0
        calibrateMe = True

        # Perform factory reset and reinitialize
        sensor.perform_factory_reset()
        print("Factory reset performed.")
        time.sleep(2)  # Wait for the factory reset to complete
        
        # Perform soft reset and reinitialize
        sensor.soft_reset()
        time.sleep(1)  # Wait for the reset to complete
        sensor.set_measurement(start=False, single_shot=False)
        time.sleep(1)
        sensor.set_measurement(start=True, single_shot=False)
        time.sleep(1)
        print("Performing soft reset and reinitialization...")

        while sensorCycle < 60:
            conversion_cycle_time = sensor.get_conversion_cycle_time()
            reading = get_sensor_reading(sensor, conversion_cycle_time)
            if reading:
                co2, temp, humidity = reading
                time.sleep(30)
                sensorCycle += 1
                print(f"Continuous Reading: CO2: {co2} ppm, Temperature: {temp:.2f} °C, Humidity: {humidity:.2f} %")
                print(f"Sensor Cycled {sensorCycle} times of 3600")
                if sensorCycle == 59:
                    if calibrateMe == True:
                        print("Initiating forced recalibration...")
                        target_co2_concentration = int(426 * 1.15)  # Mauna Loa CO2 = 426.57ppm April 2024 + 15-20% differential for being near/in a major city(Chicago), to int bc need int for low level
                        correction_value = sensor.force_recalibration(target_co2_concentration)
                        print(f"Forced recalibration completed with correction value: {correction_value} ppm")
                        sensor.set_auto_calibration(False)
                        print("Sensor Auto Calibration set to: {sensor.get_auto_calibration}. Rerun the program after power cycling with the calibration commented out to double check persistence! ")
                        sensor.save_config()
                        print("Settings saved.")
            else:
                # Reset the sensor and reinitialize if an error occurs
                print("Attempting to reinitialize the sensor...")
                sensor.set_measurement(start=False, single_shot=False)
                sensor.set_measurement(start=True, single_shot=False)
    if switch_state == 1:
        print("Skipping calibration.")
    # Endless loop to continuously read and display sensor data

    while True:
        reading = get_sensor_reading(sensor, conversion_cycle_time,last_update_sensor)
        if reading:
            co2, temp, humidity = reading
            last_update_sensor =  time.time()
            temp = temp * 9 / 5 + 32
            print(f"Continuous Reading: CO2: {co2} ppm, Temperature: {temp:.2f} °F, Humidity: {humidity:.2f} %")
            # Get current date and time
            current_time = get_time_chicago()
            date_str = "{:04}-{:02}-{:02}".format(current_time[0], current_time[1], current_time[2])
            time_str = "{:02}:{:02}:{:02}".format(current_time[3], current_time[4], current_time[5])

            data = {
            'date': date_str, 
            'time': time_str, 
            'co2': co2,
            'temp_f': temp,
            'humidity': humidity
        }
            
            send_data_to_google_sheets(data)
            thingspeak_buffer.append(data)  # Add data to buffer
            send_data_to_thingspeak()
            time.sleep(25)
#         else:
#             # Reset the sensor and reinitialize if an error occurs
#             print("Attempting to reinitialize the sensor...")
#             sensor.set_measurement(start=False, single_shot=False)
#             sensor.set_measurement(start=True, single_shot=False)

if __name__ == '__main__':
    main()


