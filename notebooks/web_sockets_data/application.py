import time
import json
import random
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit

REFRESH_RATE = 20  # Hz
UPDATE_FREQUENCY = 1 / REFRESH_RATE

app = Flask(__name__)
socketio = SocketIO(app)

# Maintain a pool of connected clients sid (Session ID)
clients = set()

# Store each clients data ({sid: data})
client_data = {}


@app.route('/')
def index():
    return render_template('index_old.html')


@socketio.on('connect')
def handle_connect():
    clients.add(request.sid)
    emit('update_pool', list(clients), broadcast=True)

    print(f'Client {request.sid} connected')
    print(f'Clients list: {clients}')

    # Initialize the random number for the new client
    client_data[request.sid] = {'settings': {}, 'data': {}}
    emit('real_time_data', client_data[request.sid]['data'], room=request.sid)


@socketio.on('disconnect')
def handle_disconnect():
    print(f'Client {request.sid} disconnected')
    clients.remove(request.sid)
    emit('update_pool', list(clients), broadcast=True)

    # Remove the client's data when they disconnect
    del client_data[request.sid]


@socketio.on('client_data')
def handle_client_data(settings):
    client_data[request.sid]['settings'] = settings
    print(f'Received {settings} from client {request.sid}')
    # emit('real_time_data', client_data[request.sid], room=request.sid)


def generate_random_polygon_json(num_polygons=100):
    polygons = []

    for _ in range(num_polygons):
        polygon = {
            "points": [
                {"x": random.uniform(0, 100), "y": random.uniform(0, 100)},
                {"x": random.uniform(0, 100), "y": random.uniform(0, 100)},
                {"x": random.uniform(0, 100), "y": random.uniform(0, 100)},
                {"x": random.uniform(0, 100), "y": random.uniform(0, 100)},
            ]
        }
        polygons.append(polygon)

    json_data = {"polygons": polygons}
    # return json.dumps(json_data, indent=2)
    return json.dumps(json_data)


def send_real_time_data():
    while True:
        time.sleep(UPDATE_FREQUENCY)  # Adjust the interval as needed

        # Update and broadcast the global random number for each client
        for client_sid in client_data:
            client_data[client_sid]['data'] = generate_random_polygon_json()
            # print(f'Generating data for client {client_sid}: {client_data[client_sid]["sid"]}')
            socketio.emit('real_time_data', client_data[client_sid]['data'], room=client_sid)


if __name__ == '__main__':
    socketio.start_background_task(target=send_real_time_data)
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
