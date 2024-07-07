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
    
def on_connection(client, userdata, flags, rc):
    print(userdata + " Connected with result code "+str(rc))  
    
    
def create_mqtt_client(name="default", on_message=None, sub_topic=None):
    m_client = mqtt.Client()
    m_client.connect("localhost", 1883)
    if on_message is not None:
        m_client.on_message = on_message
    m_client.on_connect = on_connection
    m_client.user_data_set(name)
    if sub_topic is not None:
        for topic in sub_topic:
            m_client.subscribe(topic)
    m_client.loop_start()
    return m_client
    
