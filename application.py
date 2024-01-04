#                      ___          ___          ___          ___        ___
#                     /\  \        /\  \        /\  \        /\  \      /\  \
#                    /::\  \      /::\  \      /::\  \      /::\  \     \:\  \
#                   /:/\:\  \    /:/\:\  \    /:/\:\  \    /:/\:\  \     \:\  \
#                  /::\~\:\  \  /:/  \:\  \  /::\~\:\__\  /:/  \:\  \    /::\  \
#                 /:/\:\ \:\__\/:/__/ \:\__\/:/\:\ \:|__|/:/__/ \:\__\  /:/\:\__\
#                 \/_|::\/:/  /\:\  \ /:/  /\:\~\:\/:/  /\:\  \ /:/  / /:/  \/__/
#                    |:|::/  /  \:\  /:/  /  \:\ \::/  /  \:\  /:/  / /:/  /
#                    |:|\/__/    \:\/:/  /    \:\/:/  /    \:\/:/  /  \/__/
#                    |:|  |       \::/  /      \::/  /      \::/  /
#                     \|__|        \/__/        \/__/        \/__/
#
#        ___                     ___          ___          ___   ___       ___          ___          ___
#       /\  \         ___       /\__\        /\__\        /\__\ /\  \     /\  \        /\  \        /\  \
#      /::\  \       /\  \     /::|  |      /:/  /       /:/  //::\  \    \:\  \      /::\  \      /::\  \
#     /:/\ \  \      \:\  \   /:|:|  |     /:/  /       /:/  //:/\:\  \    \:\  \    /:/\:\  \    /:/\:\  \
#    _\:\~\ \  \     /::\__\ /:/|:|__|__  /:/  /  ___  /:/  //::\~\:\  \   /::\  \  /:/  \:\  \  /::\~\:\  \
#   /\ \:\ \ \__\ __/:/\/__//:/ |::::\__\/:/__/  /\__\/:/__//:/\:\ \:\__\ /:/\:\__\/:/__/ \:\__\/:/\:\ \:\__\
#   \:\ \:\ \/__//\/:/  /   \/__/~~/:/  /\:\  \ /:/  /\:\  \\/__\:\/:/  //:/  \/__/\:\  \ /:/  /\/_|::\/:/  /
#    \:\ \:\__\  \::/__/          /:/  /  \:\  /:/  /  \:\  \    \::/  //:/  /      \:\  /:/  /    |:|::/  /
#     \:\/:/  /   \:\__\         /:/  /    \:\/:/  /    \:\  \   /:/  / \/__/        \:\/:/  /     |:|\/__/
#      \::/  /     \/__/        /:/  /      \::/  /      \:\__\ /:/  /                \::/  /      |:|  |
#       \/__/                   \/__/        \/__/        \/__/ \/__/                  \/__/        \|__|

# ---------------------------------- imports --------------------------------- #

# Libraries
import sys
import time
import logging
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
import threading

# Local imports
from model.world.world import World

# ---------------------------------- config ---------------------------------- #

# Configure the logger
logging.basicConfig(stream=sys.stdout, level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# Configure the app
app = Flask(__name__)
socketio = SocketIO(app)

# Maintain a pool of connected clients sid (Session ID)
clients = set()

# Store each clients data ({sid: data})
client_data = {}

# Maintain a lock to synchronize the access to clients data
clients_lock = threading.Lock()

# Application refresh rate
# 20 Hz = 20 times a second: 1/20 = 0.05 update interval
REFRESH_RATE = 20  # Hz
UPDATE_FREQUENCY = 1 / REFRESH_RATE

# -------------------------- routes and websockets --------------------------- #


@app.route('/')
def index():
    return render_template('index.html')


@socketio.on('connect')
def handle_connect():
    """
    Handle new client connection
    """

    with clients_lock:

        # Add the new user
        clients.add(request.sid)

        # For each client we need:
        # 1. Simulation settings (sim_settings): what to show, preferences and so on
        # 2. Simulation controls (sim_controls): if the simulation is currently playing/stopped/being reset
        # 3. World (data)
        client_data[request.sid] = {}

        # Dictionary containing this user's preferences for the simulation
        client_data[request.sid]['sim_settings'] = {
            'show_path': True,
            'show_data_structures': True,
            'autostart': True,  # If True, automatically restart the planning sequence after the map is reset
        }

        # Dictionary containing this user's simulation control variables
        client_data[request.sid]['sim_controls'] = {
            'running': False,  # True once the play button is pressed
            'stepping': True,  # True when the stepping button is pressed, set to false automatically after one iteration
        }

        # Initialize the world for the new client
        client_data[request.sid]['data'] = World(UPDATE_FREQUENCY)

        # Log the new connection
        logger.info(f'Client {request.sid} connected')
        logger.info(f'Clients list: {clients}')


@socketio.on('disconnect')
def handle_disconnect():
    """
    Remove the client's data when they disconnect
    """

    with clients_lock:

        sid = request.sid
        clients.remove(sid)
        del client_data[sid]
        logger.info(f'Client {sid} disconnected')


def send_world_data(sid):
    """
    Emit world data for session ID
    """

    world = client_data[sid]['data']
    emit('real_time_data', world.to_json(), room=sid)


@socketio.on('simulation_settings_update')
def handle_simulation_settings_update(new_settings):
    """
    Handle simulation setting update request from the client.
    'new_settings' is a dictionary containing setting: value pairs
    """

    sid = request.sid
    current_settings = client_data[sid]['sim_settings']
    for key, value in new_settings.items():
        if key in current_settings:

            # Set the new value
            current_settings[key] = value
            logger.info(f'User {sid} settings status update: {sid}')
        else:
            logger.info(f'Invalid settings status update: {key}, {value}')

    # If we change a visual preference (e.g. show the path) we need to see
    # the changes without stepping the simulation
    send_world_data(sid)


@socketio.on('simulation_controls_update')
def handle_simulation_controls_update(command):
    """
    Handle simulation control update request from the client.
    'command' is a string in ['play', 'stop', 'step', 'reset']
    """

    sid = request.sid
    sim_control = client_data[sid]['sim_controls']
    if command == 'play':
        sim_control['running'] = True
    elif command == 'stop':
        sim_control['running'] = False
    elif command == 'step':
        sim_control['stepping'] = True
    elif command == 'reset':

        sim_control['running'] = False
        sim_control['stepping'] = True

        if sim_control['autostart']:
            sim_control['running'] = True
            sim_control['stepping'] = False

        world = client_data[sid]['data']
        world.reset_robots()
        world.map.reset_map()

    # No need to send world data as these booleans will affect (stop/resume) the next iteration
        
    logger.info(f'User {sid} simulation control update: {command}')


@socketio.on('map_io_command')
def handle_map_io(command, data=None):
    """
    Handle map input/output request. Available requests to the map are 'load' and 'random'.
    If the command is 'load', the server will use 'data' as the new map.
    If the command is 'random', the server will discard 'data' and randomly generate a new map.
    The frontend will handle the logic to save a map to the client's machine.
    """

    sid = request.sid
    world = client_data[sid]['data']
    sim_settings = client_data[sid]['sim_settings']
    sim_control = client_data[sid]['sim_controls']

    if command == 'load':
        if data is None:
            raise ValueError(f'Invalid map data: {data}')

        world.map.load_map_from_json_data(data)

    elif command == 'random':

        world.map.clear()
        world.map.generate(world.robots)

    else:
        raise ValueError(f'Invalid map IO request: {command}')

    # Reset robots and their controller
    world.reset_robots()

    # Step the simulation
    sim_control['running'] = False
    sim_control['stepping'] = True

    if sim_settings['autostart']:
        sim_control['running'] = True
        sim_control['stepping'] = False

    logger.info(f'User {sid} map IO command: {command}')


@socketio.on('world_status_update')
def handle_world_status_update(status_update):
    """
    Handle world status update request from the client.
    World status update include:
    1. robots linear speed
    2. robots angular speed
    3. obstacles spawn frequency
    'status_update' is a dictionary containing variable: value pairs
    """

    sid = request.sid
    world = client_data[sid]['data']

    if 'obstacles_spawn_frequency' in status_update:
        spawn_frequency = float(status_update['obstacles_spawn_frequency'])
        logger.info(f'User {sid} world status update: obstacles_spawn_frequency={spawn_frequency}')
        # TODO use this value

    if 'robots_linear_velocity' in status_update:
        linear_velocity = float(status_update['robots_linear_velocity'])
        logger.info(f'User {sid} world status update: robots_linear_velocity={linear_velocity}')
        for robot in world.robots:
            robot.linear_velocity = linear_velocity

    if 'robots_angular_velocity' in status_update:
        angular_velocity = float(status_update['robots_angular_velocity'])
        logger.info(f'User {sid} world status update: robots_angular_velocity={angular_velocity}')
        for robot in world.robots:
            robot.angular_velocity = angular_velocity


def send_real_time_data():

    while True:
        time.sleep(UPDATE_FREQUENCY)

        with clients_lock:

            for client_sid in client_data:

                world = client_data[client_sid]['data']
                sim_control = client_data[client_sid]['sim_controls']

                # Step the simulation
                world.step()

                # Emit new data
                socketio.emit('real_time_data', world.to_json(), room=client_sid)

                if sim_control['stepping']:
                    sim_control['stepping'] = False


if __name__ == '__main__':
    socketio.start_background_task(target=send_real_time_data)
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
