import requests

def send_tracking_data(angle):
    url = 'http://localhost:8080/tracking_data'
    payload = {'z': angle}
    headers = {'Content-Type': 'application/json'}

    response = requests.post(url, json=payload, headers=headers)

    if response.status_code == 200:
        print("Angle sent successfully")
    else:
        print("Error sending angle:", response.status_code)

while True:
    send_tracking_data(30)