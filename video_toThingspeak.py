# Thingspeak
import requests



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
    # SEND to Thingspeak
    toThingspeak = [55 , 66 , 77 , 88 , 99 ]
    api_key = "EKNZFX5H01RUTCPS"
    
    
    send_to_thingspeak(api_key,toThingspeak)
    
