# DHT 11 imports
import board
import adafruit_dht

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

if __name__ == "__main__":
    dht_reader = DHT11Reader()
    temp = dht_reader.get_temp()
    print(temp)