#Arduino
import serial
import time


class ArduinoReader:
    def __init__(self, port):
        """
        Initializes the Arduino Reader.
        """
        self.ser = self.open_serial_port(port, 9600)  # Change 'COM8' to your serial port
        self.sensor1 = None
        self.sensor2 = None
        
    def open_serial_port(self, port, baudrate):
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
            recLines = []
            while self.ser.in_waiting > 0:
                fullLine = self.ser.readline()
                recLines.append(fullLine)
            if len(recLines) > 0:                
                lastLine = recLines[len(recLines)-1]
                #print(len(recLines))
                data = lastLine.decode('utf-8').strip()
                parts = data.split("\t")
                for part_ in parts:
                    part = part_.strip()
                    if part.startswith('sensor1'):
                        self.sensor1 = int(part.split('=')[1].strip())
                    elif part.startswith('sensor2'):
                        self.sensor2 = int(part.split('=')[1].strip())
        except serial.SerialException as e:
            print(f"Error: Serial port communication error: {e}")
        
    def get_Sensor(self, sensornum):
        #sensor1, sensor2 = self.read_sensors()
        self.process_Serial()
        if sensornum == 1:
            return self.sensor1
        else:
            return self.sensor2
        
        
if __name__ == "__main__":
    #dht_reader = DHT11Reader()
    arduino = ArduinoReader('/dev/ttyACM0')
    time.sleep(0.75)
    
    potmeter = arduino.get_Sensor(1)
    lightRaw = arduino.get_Sensor(2)
   
    print("Arduino potmeter    : {}".format(potmeter))
    print("Arduino Light sensor: {}".format(lightRaw))
    