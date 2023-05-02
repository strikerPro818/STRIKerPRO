from src.rmd_x8 import RMD_X8
from flask import Flask, request, jsonify
from flask_socketio import SocketIO

robot = RMD_X8(0x141)
robot.setup()

app = Flask(__name__)
socketio = SocketIO(app)
desired_yaw_speed = 80



@app.route('/tracking_data', methods=['POST'])
def handle_tracking_data():
    if request.method == 'POST':
        payload = request.get_json()

        # Check the payload and print it
        if 'z' in payload:
            angle = payload['z']
            print(f"Received angle: {angle}")
            panAngle(angle)
        else:
            print("Payload does not contain 'z' key")

        return jsonify({'status': 'success'}), 200
    else:
        return jsonify({'status': 'method not allowed'}), 405

def panAngle(angle):
    global desired_yaw_speed
    target = int(angle * 100)
    angle_bytes = target.to_bytes(4, 'little', signed=True)
    speed_bytes = desired_yaw_speed.to_bytes(2, 'little', signed=True)
    data = list(speed_bytes) + list(angle_bytes)
    print("Executed Angle:", angle)
    robot.position_closed_loop_2(data)


if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=9090, allow_unsafe_werkzeug=True)

