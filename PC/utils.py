import requests
import paho.mqtt.client as mqtt


def send_post_request(target_ip, param=None):
    if param is None:
        param = {}
    try:
        response = requests.post(target_ip, json=param, timeout=5)
        if response.status_code != 200:  # Check if the response status code is not 200 (OK)
            print(f"Error sending request: {response.status_code}")
        #print(f"Response code: {response.status_code}")
        return response
    except requests.exceptions.RequestException as e:
        print(f"Error sending request: {e}")
        return None
    
def send_get_request(target_ip, params=None):
    try:
        response = requests.get(target_ip, params=params, timeout=5)
        if response.status_code != 200:
            print(f"Error sending request: {response.status_code}")
        return response
    except requests.exceptions.RequestException as e:
        print(f"Error sending request: {e}")
        return None

# Default connection callback
def on_connection(client : mqtt.Client, userdata, flags, rc):
    print(userdata["name"] + " Connected with result code " + str(rc))
    if rc == 0 and 'sub_topic' in userdata:
        if isinstance(userdata["sub_topic"], list):
            sub_topic_tuple_list = []
            for topic in userdata["sub_topic"]:
                sub_topic_tuple_list.append((topic, 0))
            print(f"{userdata['name']} : Subscribed to topics: {sub_topic_tuple_list}")
            client.subscribe(sub_topic_tuple_list)
        elif userdata["sub_topic"] is not None:
            print(f"{userdata['name']} : Subscribed to topic: {userdata['sub_topic']}")
            client.subscribe((userdata["sub_topic"], 0))
    else:
        print("Failed to connect, return code %d\n", rc)

# Default message callback
def on_message_default(client, userdata, msg):
    print(f"{userdata['name']} msg get: {msg}")

# Function to create MQTT client
def create_mqtt_client(name="default", on_message=on_message_default, sub_topic=None):
    m_client = mqtt.Client()
    userdata = {'name': name, 'sub_topic': sub_topic}
    m_client.user_data_set(userdata)
    m_client.on_connect = on_connection
    m_client.on_message = on_message
    m_client.connect("localhost", 1883, 60)  # Added keepalive parameter
    m_client.loop_start()
    return m_client
