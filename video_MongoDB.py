#MONGODB
from pymongo import MongoClient
from datetime import datetime

class SensorDataStore:
    def __init__(self, db_name, collection_name, host, port=27017, reconnect_attempts=3, reconnect_delay=5):
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
            print(f"Sensor data point for sensor {sensor_id} inserted successfully.")
        except Exception as e:
            print(f"Failed to insert data for sensor {sensor_id}: {e}")
            self.connect()  # Attempt to reconnect and retry insertion

if __name__ == "__main__":
    data_store = SensorDataStore('iotdb','time_test','10.0.0.24')
    
    #Create fake data
    dataFrame = {}
    dataFrame['light_arduino'] = 111
    dataFrame['potmeter_arduino'] = 999
    
    #SendToMongoDB
    for sensor_id, value in dataFrame.items():
        data_store.insert_sensor_data(sensor_id, value)
    
    
