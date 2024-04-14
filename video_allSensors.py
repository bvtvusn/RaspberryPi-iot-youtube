import time

# 3002 imports:
import spidev

# DHT 11 imports
import board
import adafruit_dht

#TC74
import smbus

#Arduino
import serial

#MONGODB
from pymongo import MongoClient
from datetime import datetime

# LED and button
import pigpio

# Thingspeak
import requests


class LowPassFilter:
    def __init__(self, timestep, timeconstant):
        """
        Initializes the Low Pass Filter.
        :param timestep: The time step of the system.
        :param timeconstant: The time constant of the filter.
        """
        self.timestep = timestep
        self.timeconstant = timeconstant
        self.out_prev = 0

    def apply_filter(self, indata):
        """
        Applies low pass filter to input data.
        :param indata: Input data to filter.
        :return: Filtered output data.
        """
        filterCoefficient = self.timestep / (self.timestep + self.timeconstant)
        output = (1 - filterCoefficient) * self.out_prev + filterCoefficient * indata
        self.out_prev = output
        return output
    def get_state(self):
        return self.out_prev

class ADCReader:
    def __init__(self, spi_channel=0, vref=3.3, max_speed_hz=1200000):
        """
        Initializes the ADC Reader.
        :param spi_channel: SPI channel to communicate with ADC.
        :param vref: Voltage reference for ADC.
        :param max_speed_hz: Maximum SPI speed.
        """
        self.spi_channel = spi_channel
        self.spi = spidev.SpiDev(0, spi_channel)
        self.spi.max_speed_hz = max_speed_hz
        self.vref = vref

    def read_adc(self, adc_channel):
        """
        Reads ADC value from specified channel.
        :param adc_channel: Channel number to read from.
        :return: ADC value.
        """
        if adc_channel != 0:
            adc_channel = 1

        msg = 0b11
        msg = ((msg << 1) + adc_channel) << 5
        msg = [msg, 0b00000000]
        reply = self.spi.xfer2(msg)

        adc = 0
        for n in reply:
            adc = (adc << 8) + n

        adc = adc >> 1
        voltage = (self.vref * adc) / 1024

        return voltage

    def get_value(self):
        """
        Reads ADC value from MCP3002 and converts it to temperature.
        :return: Temperature in Celsius.
        """
        try:
            self.spi.open(0, self.spi_channel)
            adc_0 = self.read_adc(0)
            MCPtemp = (adc_0 - 0.5) * 100  # MCP9700 Has DC offset of 500mV and 10mV per degree Celsius.
            return MCPtemp
        finally:
            self.spi.close()
            
class DHT11Reader:
    def __init__(self):
        """
        Initializes the DHT11 Reader.
        """
        self.sensor = adafruit_dht.DHT11(board.D4, use_pulseio = False)
        
    def get_temp(self):
        dhtTemp = None
        try:
            # Print the values to the serial port
            dhtTemp = self.sensor.temperature           
        except Exception as error:
            # Errors happen fairly often, DHT's are hard to read, just keep going
            print(error)        
        return dhtTemp
        
        
class TC74Reader:
    def __init__(self):
        """
        Initializes the DHT11 Reader.
        """
        # I2C address of the TC74 sensor
        self.TC74_ADDRESS = 0x48
        # Register address to read temperature
        self.TEMP_REG = 0x00
        # Create I2C bus
        self.bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1, for Raspberry Pi 3/4. Use 0 for older models.
                
    def get_temp(self):
        # Read temperature data from sensor
        TCtemp_data = self.bus.read_byte_data(self.TC74_ADDRESS, self.TEMP_REG)
        # Convert temperature data to Celsius
        TCtemperature = TCtemp_data if TCtemp_data < 128 else TCtemp_data - 256
        return TCtemperature
        
        
class ArduinoReader:
    def __init__(self):
        """
        Initializes the Arduino Reader.
        """
        self.ser = self.open_serial_port('/dev/ttyACM0', 9600)  # Change 'COM8' to your serial port
        self.sensor1 = None
        self.sensor2 = None
        # Models.
    def open_serial_port(self, port, baudrate):
        print(port)
        try:
            self.ser = serial.Serial(port, baudrate)
            return self.ser
        except serial.SerialException as e:
            print(f"Error: Unable to open serial port '{port}': {e}")
            return None
    def close_serial_port(self, ser):
        try:
            self.ser.close()
            print("Serial port closed.")
        except serial.SerialException as e:
            print(f"Error: Unable to close serial port: {e}")
    def process_Serial(self):
                
        try:
            while self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').strip()
                parts = data.split("\t")
                for part_ in parts:
                    part = part_.strip()
                    if part.startswith('sensor1'):
                        self.sensor1 = int(part.split('=')[1].strip())
                    elif part.startswith('sensor2'):
                        self.sensor2 = int(part.split('=')[1].strip())
        except serial.SerialException as e:
            print(f"Error: Serial port communication error: {e}")
        
        #return sensor1, sensor2
    
    def get_Sensor(self, sensornum):
        #sensor1, sensor2 = self.read_sensors()
        self.process_Serial()
        if sensornum == 1:
            return self.sensor1
        else:
            return self.sensor2
        
class SensorDataStore:
    def __init__(self, db_name='iotdb', collection_name='time_test', host='10.0.0.24', port=27017, reconnect_attempts=3, reconnect_delay=5):
        """
        Initialize the SensorDataStore with MongoDB connection details.
        Args:
            db_name (str): Name of the database.
            collection_name (str): Name of the collection.
            host (str): MongoDB server host address.
            port (int): MongoDB server port.
            reconnect_attempts (int): Number of attempts to reconnect on connection failure.
            reconnect_delay (int): Delay between reconnection attempts (in seconds).
        """
        self.db_name = db_name
        self.collection_name = collection_name
        self.host = host
        self.port = port
        self.reconnect_attempts = reconnect_attempts
        self.reconnect_delay = reconnect_delay
        self.connect()

    def connect(self):
        """
        Connect to MongoDB.
        """
        attempt = 1
        while True:
            try:
                self.client = MongoClient(self.host, self.port)
                self.db = self.client[self.db_name]
                self.collection = self.db[self.collection_name]
                print("Connected to MongoDB successfully.")
                break
            except Exception as e:
                if attempt <= self.reconnect_attempts:
                    print(f"Connection failed (Attempt {attempt}/{self.reconnect_attempts}). Retrying in {self.reconnect_delay} seconds...")
                    attempt  = 1
                    sleep(self.reconnect_delay)
                else:
                    print(f"Failed to connect to MongoDB after {self.reconnect_attempts} attempts.")
                    raise e

    def insert_sensor_data(self, sensor_id, value):
        """
        Insert sensor data into the MongoDB collection.
        Args:
            sensor_id (str): ID of the sensor.
            value (float|int): Value recorded by the sensor.
        """
        timestamp = datetime.utcnow()
        data_point = {'timestamp': timestamp, 'sensor_id': sensor_id, 'value': value}
        try:
            self.collection.insert_one(data_point)
            #print(f"Sensor data point for sensor {sensor_id} inserted successfully.")
        except Exception as e:
            print(f"Failed to insert data for sensor {sensor_id}: {e}")
            self.connect()  # Attempt to reconnect and retry insertion



def setBrightness(_gpio, percentage):
    FREQUENCY=500
    f = pi.set_PWM_frequency(_gpio, FREQUENCY)
    norm = percentage / 100.0
    r = pi.get_PWM_range(_gpio)
    pi.set_PWM_dutycycle(_gpio, r * norm)

def send_to_thingspeak(api_key, field_values):
    """
    Send data to ThingSpeak.

    Args:
        api_key (str): Your ThingSpeak API key.
        field_values (list): List containing field values.
    """
    url = f"https://api.thingspeak.com/update?api_key={api_key}"
    for i, value in enumerate(field_values, start=1):
        url += f"&field{i}={value}"
    response = requests.get(url)
    if not response.status_code == 200:
        print(f"Failed to send data to ThingSpeak. Status code: {response.status_code}")
    else:
        print("Data sent to ThingSpeak successfully.")


if __name__ == "__main__":
    adc_reader = ADCReader()
    AdcFilter = LowPassFilter(1.0, 3.0)
    
    dht_reader = DHT11Reader()
    DhtFilter = LowPassFilter(1.0, 3.0)
    
    TC74_reader = TC74Reader()
    TC74Filter = LowPassFilter(1.0, 3.0)
    
    arduino = ArduinoReader()
    lightFilter = LowPassFilter(1.0, 3.0)
    
    data_store = SensorDataStore('iotdb','time_test')
    dataFrame = {}
    
    
    #BUTTON and LED
    GPIO_led=12
    pi = pigpio.pi() # Connect to local Pi.
    
    GPIO_btn = 5
    pi.set_mode(GPIO_btn, pigpio.INPUT)
    pi.set_pull_up_down(GPIO_btn, pigpio.PUD_UP)  # Assuming the button is connected to ground when pressed



    # ALARMING
    alarm_state = 0  
    #   0           1                   2          
    # off, active-unacknowledged, active-ack
    
    
    
    time.sleep(1) # wait for serial data
    
    
    try:
        while True:  

            dataFrame = {}
            
            
            temp = adc_reader.get_value()        
            filteredAdc = AdcFilter.apply_filter(temp)
            print("ADC: {:.2f}, Filtered: {:.2f}".format(temp, filteredAdc))
            dataFrame['temp_filt_MCP9700'] = filteredAdc
            
            
            dhtRaw = dht_reader.get_temp()
            #print(type(dhtRaw))
            if dhtRaw is not None:
                DhtFilter.apply_filter(dhtRaw)
                #print(dhtRaw)
            dht_filtered = DhtFilter.get_state()
            print("DHT Filtered: {:.2f}".format(dht_filtered))
            dataFrame['temp_filt_dht11'] = dht_filtered
                
                
            TCtemp = TC74_reader.get_temp()
            TCfilt = TC74Filter.apply_filter(TCtemp)
            print("TC74 Filtered: {:.2f}".format(TCfilt))
            dataFrame['temp_filt_tc74'] = TCfilt
            
            
            arduinoTemp = arduino.get_Sensor(1)            
            if arduinoTemp is not None:
                print("Potmeter: {:.2f}".format(arduinoTemp))
                dataFrame['potmeter_arduino'] = arduinoTemp
            
            
            lightRaw = arduino.get_Sensor(2)
            if lightRaw is not None:
                lightFilter.apply_filter(lightRaw)
            
            print("Light Sensor: {:.2f}".format(lightRaw))
            dataFrame['light_arduino'] = lightRaw
            
            # Read alarm acknowledge button
            button_state = pi.read(GPIO_btn)
            AckButtonpressed = (button_state != 0)
            
            #dataFrame['light_arduino'] =999
            # Alarming
            alarmlimit = 300
            if alarm_state == 0:
                setBrightness(GPIO_led,0)
                if dataFrame['light_arduino'] < alarmlimit:
                
                    alarm_state = 1 # Activate alarm if below alarm limit
            if alarm_state == 1:
                setBrightness(GPIO_led,100)
                if AckButtonpressed:
                    alarm_state = 2 # Acknowledging
            if alarm_state == 2:
                setBrightness(GPIO_led,20)
                if dataFrame['light_arduino'] > alarmlimit:
                    alarm_state = 0 # set to Cleared
            print("Alarm state: {:.2f}".format(alarm_state))
            
            
            #SendToMongoDB
            for sensor_id, value in dataFrame.items():
                data_store.insert_sensor_data(sensor_id, value)
                
            # SEND to Thingspeak
            toThingspeak = [dataFrame['temp_filt_MCP9700'] , dataFrame['temp_filt_dht11'] , dataFrame['temp_filt_tc74'] , dataFrame['potmeter_arduino'] , dataFrame['light_arduino'] , ]
            api_key = "EKNZFX5H01RUTCPS"
            send_to_thingspeak(api_key,toThingspeak)
                
            
            
            
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Exiting...")
        
        pi.stop()
