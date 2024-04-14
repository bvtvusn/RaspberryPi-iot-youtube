# 3002 imports:
import spidev

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
    

if __name__ == "__main__":
    ADCreader = ADCReader()
    temp = ADCreader.get_value()
    print(temp)