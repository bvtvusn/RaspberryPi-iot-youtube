import paho.mqtt.client as mqtt # pip3 install paho-mqtt==1.6.1
import time

class MQTTUploader:
    def __init__(self, broker_address, port):
        self.broker_address = broker_address
        self.port = port
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(self.broker_address, self.port)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected with result code " + str(rc))

    def upload_values(self, path, value):
        self.client.publish(path, str(value))
        print("Published value:", value)

# Example usage:
if __name__ == "__main__":
    broker_address = "test.mosquitto.org"
    port = 1883
    
    uploader = MQTTUploader(broker_address, port)
    while True:
        uploader.upload_values("bvlabs/iotData/dht11", 59) 
        time.sleep(1)
        