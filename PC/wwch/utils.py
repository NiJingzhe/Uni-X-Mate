import requests
def send_post_request(target_ip, params=None):
    if params is None:
        params = {}
    try:
        response = requests.post(target_ip, json=params, timeout=5)
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
