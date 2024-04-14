#TC74
import smbus

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
       


if __name__ == "__main__":
    TC74_reader = TC74Reader()
    TCtemp = TC74_reader.get_temp()
    print(TCtemp)
