import time
import random
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
import threading

app = Flask(__name__)
socketio = SocketIO(app)

# Maintain a pool of connected clients sid (Session ID)
clients = set()

# Store each clients data ({sid: data})
client_data = {}

# Maintain a lock to synchronize the access to clients data
clients_lock = threading.Lock()


@app.route('/')
def index():
    return render_template('index_old.html')


@socketio.on('connect')
def handle_connect():
    print(f'Client {request.sid} connected')
    with clients_lock:
        clients.add(request.sid)
        emit('update_pool', list(clients), broadcast=True)
        # emit('your_sid', request.sid)

        # Initialize the random number for the new client
        client_data[request.sid] = {'yourNumber': random.randint(1, 100), 'globalNumber': None}
        emit('real_time_data', client_data[request.sid], room=request.sid)


@socketio.on('disconnect')
def handle_disconnect():
    print(f'Client {request.sid} disconnected')
    with clients_lock:
        clients.remove(request.sid)
        emit('update_pool', list(clients), broadcast=True)

        # Remove the client's data when they disconnect
        del client_data[request.sid]


@socketio.on('send_your_number')
def handle_send_your_number(data):
    client_data[request.sid]['yourNumber'] = data['yourNumber']
    print(f'Received {data["yourNumber"]} from client {request.sid}')
    emit('real_time_data', client_data[request.sid], room=request.sid)


def send_real_time_data():
    while True:
        time.sleep(1)  # Adjust the interval as needed

        # Update and broadcast the global random number for each client
        with clients_lock:
            for client_sid in client_data:
                random_number = random.randint(1, 100)
                client_data[client_sid]['globalNumber'] = random_number
                socketio.emit('real_time_data', client_data[client_sid], room=client_sid)


if __name__ == '__main__':
    socketio.start_background_task(target=send_real_time_data)
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
