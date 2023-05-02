from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('/tracking_data', methods=['POST'])
def handle_tracking_data():
    if request.method == 'POST':
        payload = request.get_json()

        # Check the payload and print it
        if 'z' in payload:
            angle = payload['z']
            print(f"Received angle: {angle}")
        else:
            print("Payload does not contain 'z' key")

        return jsonify({'status': 'success'}), 200
    else:
        return jsonify({'status': 'method not allowed'}), 405

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)
