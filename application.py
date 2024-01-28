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
import importlib
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
import threading
from typing import Literal

# Local imports
from model.geometry.circle import Circle
from model.geometry.point import Point

from model.world.world import World
from model.world.map.map_builder import MapBuilder


from model.world.robot.differential_drive_robot import DifferentialDriveRobot
from model.world.robot.robots.cobalt.cobalt import Cobalt
from model.world.robot.URDF_parser import URDFParser

# Controller and search algorithms
from model.controllers.controller import Controller
from model.controllers.search_based.AStar import AStar
from model.controllers.sampling_based.DynamicRRT import DynamicRRT
from model.controllers.sampling_based.RRTStar import RRTStar

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

"""
The app uses the following channels for one-time data exchange:

- simulation_control_update: Handles messages related to simulation control (start, stop, step, restart).
- simulation_settings_update: Handles messages related to simulation settings (enable autostart, show structures, ...).
- robot_update: Handles updates about the robot's status (angular speed, linear speed, load custom robot).
- map_update: Manages updates related to the map or environment (load custom map, store current map, get random map).
- controller_update: Handles messages related to the controller (algorithm).
- obstacle_control: Handles messages related to obstacle management (add, remove obstacles).

The type of messages accepted by each channel is explained below:

- simulation_control_update: string
- simulation_settings_update: {setting: value}
- robot_update: {setting: value}
- map_update: {setting: value}
- controller_update: string
- obstacle_control: float, float

Each channel requires a handler:

- handle_simulation_control_update(command)
- handle_simulation_settings_update(update_dict)
- handle_robot_update(update_dict)
- handle_map_update(update_dict)
- handle_controller_update(algorithm)
- handle_obstacle_control(x, y)

The app uses the following channels for real-time data transfer:

- real_time_data: Contains the json representation of the world, sent from the backend to the frontend.
"""

"""
For each client we need:

- simulation settings (sim_settings): what to show, preferences and so on
- simulation control (sim_control): if the simulation is currently playing/stopped/stepping/being reset
- world (data)
"""


@app.route('/')
def index():
    """
    Render the main page of the app
    """
    return render_template('index.html')


@socketio.on('connect')
def handle_connect():
    """
    Handle new client connection
    """

    with clients_lock:
        # Add the new client
        clients.add(request.sid)

        # Client dictionary
        client_data[request.sid] = {}

        # Dictionary containing this client's preferences for the simulation
        client_data[request.sid]['sim_settings'] = {
            'show_path': True,
            'show_data_structures': True,
            'autostart': True,  # If True, automatically restart the planning sequence after the map is reset
        }

        # Dictionary containing this client's simulation control variables
        client_data[request.sid]['sim_control'] = {
            'running': False,  # True once the play button is pressed
            'stepping': False,  # True when the stepping button is pressed, set false automatically after one iteration
        }

        # Generate the world
        world = World(UPDATE_FREQUENCY)

        # Generate the robot(s)
        robots = [
            Cobalt()
        ]

        # Initialize the map using a map builder
        world_map = (MapBuilder()
                     .set_obs_count(40)
                     .set_map_boundaries((-5.0, -5.0, 5.0, 5.0))
                     .set_data_structure("quadtree")
                     .build())

        # Generate a forbidden circle for each robot
        forbidden_zones = [Circle(robot.current_pose.x, robot.current_pose.y, robot.outline.radius + 0.2) for robot in robots]

        # Generate a map
        world_map.generate(forbidden_zones)

        # Take a controller
        controllers = [
            Controller(robot, AStar(world_map, robot.current_pose.as_point())) for robot in robots
            # Controller(robot, DynamicRRT(world_map, robot.current_pose.as_point())) for robot in robots
            # Controller(robot, RRTStar(world_map, robot.current_pose.as_point())) for robot in robots
        ]

        for robot, controller in zip(robots, controllers):
            world.add_robot(robot, controller)

        world.set_map(world_map)

        client_data[request.sid]['data'] = world

        # Send world data
        send_world_data(request.sid)

        # Log the new connection
        logger.info(f'Client {request.sid} connected')
        logger.info(f'Clients: {clients}')


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
    Emit world data for session ID once
    """

    world = client_data[sid]['data']
    sim_settings = client_data[sid]['sim_settings']

    emit('real_time_data', world.to_json(
        add_path=sim_settings['show_path'],
        add_data_structures=sim_settings['show_data_structures']
    ), room=sid)


def send_real_time_data():
    """
    Background update loop periodically sending data to the frontend
    """

    while True:

        time.sleep(UPDATE_FREQUENCY)

        with clients_lock:

            for client_sid in client_data:

                world = client_data[client_sid]['data']
                sim_control = client_data[client_sid]['sim_control']
                sim_settings = client_data[client_sid]['sim_settings']

                if sim_control['stepping'] or sim_control['running']:

                    # Step the simulation
                    world.step()

                    # Emit new data
                    socketio.emit('real_time_data', world.to_json(
                        add_path=sim_settings['show_path'],
                        add_data_structures=sim_settings['show_data_structures']
                    ), room=client_sid)

                    if sim_control['stepping']:
                        sim_control['stepping'] = False


@socketio.on('simulation_control_update')
def handle_simulation_control_update(command: Literal['start', 'stop', 'step', 'reset']):
    """
    Handle simulation control update request from the client.

    Parameters:
        - command (Literal['start', 'stop', 'step', 'reset']): command for simulation control.
            'start' starts the simulation;
            'stop' stops the simulation;
            'step' runs only one step of the simulation;
            'reset' resets the simulation to the initial conditions;
    """

    sid = request.sid
    sim_control = client_data[sid]['sim_control']
    sim_settings = client_data[sid]['sim_settings']

    if command == 'start':
        sim_control['running'] = True
    elif command == 'stop':
        sim_control['running'] = False
    elif command == 'step':
        sim_control['stepping'] = True
    elif command == 'reset':

        sim_control['running'] = False
        sim_control['stepping'] = False

        if sim_settings['autostart']:
            sim_control['running'] = True
            # sim_control['stepping'] = False

        world = client_data[sid]['data']
        world.reset_robots()
        world.map.reset()

    send_world_data(sid)

    logger.info(f'Client {sid} simulation control update request: {command}')


@socketio.on('simulation_settings_update')
def handle_simulation_settings_update(update_dict: dict):
    """
    Handle simulation setting update request from the client.

    Parameters:
        - update_dict (dict): dictionary containing simulation settings. Available settings are the following:
            'show_path': whether to show the path or not;
            'show_data_structures': whether to show algorithm's data structures or not;
            'autostart': whether to automatically start the simulation after a reset or a new random initial state;
    """

    sid = request.sid
    current_settings = client_data[sid]['sim_settings']
    for key, value in update_dict.items():
        if key in current_settings:

            # Set the new value
            current_settings[key] = value
            logger.info(f'Client {sid} simulation settings update request: {key}, {value}')
        else:
            logger.info(f'Invalid settings update request: {key}, {value}')

    # If we change a visual preference (e.g. show the path) we need to see
    # the changes without stepping the simulation
    send_world_data(sid)


@socketio.on('robot_update')
def handle_robot_update(update_dict: dict):
    """
    Handle robot update request from the client.

    Parameters:
        - update_dict (dict): dictionary containing robot update data. Available keys are the following:
            'angular_speed': set the angular speed of the robot;
            'linear_velocity': set the linear speed of the robot;
            'load': load a new robot using the provided URDF data;
    """

    sid = request.sid
    world = client_data[sid]['data']

    for key, value in update_dict.items():
        if key == 'linear_velocity':
            linear_velocity = float(update_dict['linear_velocity'])
            logger.info(f'Client {sid} robot update request: {key}, {value}')
            for robot in world.robots:
                robot.linear_velocity = linear_velocity
        elif key == 'angular_velocity':
            angular_velocity = float(update_dict['angular_velocity'])
            logger.info(f'Client {sid} robot update request: {key}, {value}')
            for robot in world.robots:
                robot.angular_velocity = angular_velocity
        elif key == 'load':

            #  TODO fix bug after switching robot
            robot_polygons, estimated_wheelbase = URDFParser.parse_string(update_dict['load'])
            robot = DifferentialDriveRobot(robot_polygons, estimated_wheelbase)

            # TODO provide native multi robot support
            robot.reset(world.robots[0].current_pose)
            world.robots[0] = robot

            """
            algorithm = world.controllers[0].search_algorithm
            if isinstance(algorithm, SearchBased):
                algorithm.discretization_step = estimated_wheelbase/2
            """

            # Send world data to immediately show the new robot
            send_world_data(sid)

            logger.info(f'Client {sid} robot update request: loading new robot based on provided URDF data')
        else:
            logger.info(f'Invalid robot update request: {key}, {value}')


@socketio.on('map_update')
def handle_map_update(update_dict: dict):
    """
    Handle map update request from the client.

    Parameters:
        - update_dict (dict): dictionary containing map update data. Available keys are the following:
            'load': load a map using the provided json data;
            'random': generate a new map (value is discarded);
    """

    sid = request.sid
    world = client_data[sid]['data']
    sim_settings = client_data[sid]['sim_settings']
    sim_control = client_data[sid]['sim_control']

    for key, value in update_dict.items():
        if key == 'load':

            data = update_dict['load']
            world.from_json(data)

            # Signal to the frontend that the current algorithm has changed
            # TODO provide native multi robot support
            # emit('notify_controller_update', data['controllers'][0], room=sid)

            logger.info(f'Client {sid} map update request: loading new map based on provided json data')

        elif key == 'random':

            # Generate a forbidden circle for each robot
            forbidden_zones = [Circle(robot.current_pose.x, robot.current_pose.y, robot.outline.radius + 0.2) for robot
                               in world.robots]

            # Generate a map
            world.map.generate(forbidden_zones)

            logger.info(f'Client {sid} map update request: generating new map')
        else:
            logger.info(f'Invalid map update request: {key}, {value}')

    # Reset robots and their controller
    # world.reset_robots()
    for robot, controller in zip(world.robots, world.controllers):
        controller.reset(robot.current_pose)

    # Stop the simulation
    sim_control['running'] = False
    sim_control['stepping'] = False

    # If autostart enabled, let the simulation start instead
    if sim_settings['autostart']:
        sim_control['running'] = True
        sim_control['stepping'] = False

    # Send new world data to show new map
    send_world_data(sid)


@socketio.on('algorithm_update')
def handle_controller_update(update_dict):
    """
    Updates current controller (without changing the underlying algorithm)
    """

    sid = request.sid
    world = client_data[sid]['data']

    for key, value in update_dict.items():
        if key == 'iterations_per_step':

            # TODO provide native multi robot support
            iterations_per_step = int(update_dict['iterations_per_step'])
            world.controllers[0].search_algorithm.iterations_per_step = iterations_per_step
            logger.info(f'Client {sid} algorithm update request: setting iterations per step to [{iterations_per_step}]')

        elif key == 'expire':

            # TODO provide native multi robot support
            search_algorithm = world.controllers[0].search_algorithm

            # Only for sampling based algorithms
            if hasattr(search_algorithm, 'max_iterations'):
                search_algorithm.current_iteration = search_algorithm.max_iterations
                logger.info(
                    f'Client {sid} algorithm update request: expiring iterations')


@socketio.on('algorithm_control')
def handle_algorithm_control(algorithm):
    """
    Handle controller update request from the client.

    Parameters:
        - algorithm: string representing the algorithm to use.
    """

    sid = request.sid

    algorithm_class = None

    sub_folders = ["search_based", "sampling_based"]
    for sub_folder in sub_folders:

        try:

            # Build the full import path
            module_path = f'model.controllers.{sub_folder}.{algorithm}'

            # Try to import the module dynamically
            algorithm_module = importlib.import_module(module_path)

            # Get the class dynamically
            algorithm_class = getattr(algorithm_module, algorithm)

        except (ImportError, AttributeError) as e:
            # logger.error(f"Error handling algorithm: {str(e)}")
            pass

    # TODO provide native multi robot support
    world = client_data[sid]['data']
    world.controllers[0] = Controller(
        world.robots[0], algorithm_class(world.map, start=world.robots[0].current_pose.as_point())
    )

    send_world_data(sid)

    logger.info(f'Client {sid} controller update request: selected algorithm {algorithm}')


@socketio.on('obstacle_control')
def handle_obstacle_control(x: float, y: float, query_radius: float = 0.1):
    """
    Handle obstacle control request from the client. If no obstacle is near the input (x, y) point, add an obstacle
    following the map specification (obstacle type), otherwise remove the closest one.

    Parameters:
        - x (float): x coordinate of the query point;
        - y (float): y coordinate of the query point;
    """

    sid = request.sid
    world = client_data[sid]['data']

    # Take all the obstacles in the region
    obstacles_in_region = world.map.query_polygon(Circle(x, y, query_radius))

    # Take all the robots in the region
    robots_in_region = [robot for robot in world.robots if robot.current_pose.as_point().distance(Point(x, y)) < 0.5]

    if len(robots_in_region) > 0:
        logger.info(f'Client {sid} obstacle control request: unable to add obstacle at ({x}, {y}) - too close to robot')
    else:
        if len(obstacles_in_region) > 0:
            obstacle_id = obstacles_in_region[0]
            world.map.remove_obstacle(obstacle_id)
            logger.info(f'Client {sid} obstacle control request: removing obstacle [{obstacle_id}] at ({x}, {y})')
        else:  # No obstacle in region
            result = world.world_map.spawn_obstacle_at(Point(x, y))
            if result != -1:
                logger.info(f'Client {sid} obstacle control request: adding obstacle at ({x}, {y})')
            else:
                logger.info(f'Client {sid} obstacle control request: unable to add obstacle at ({x}, {y})')

    # Send new world data to show new obstacles
    send_world_data(sid)


@socketio.on('goal_control')
def handle_goal_control(x: float, y: float):
    """
    Handle goal control request from the client.

    Parameters:
        - x (float): x coordinate of the query point;
        - y (float): y coordinate of the query point;
    """

    sid = request.sid
    world = client_data[sid]['data']
    result = world.map.set_goal(Point(x, y), clearance=0.2)
    if result:

        for robot, controller in zip(world.robots, world.controllers):
            controller.reset(robot.current_pose)

        send_world_data(sid)

        logger.info(f'Client {sid} goal control request: moving goal to ({x}, {y})')
    else:
        logger.info(f'Client {sid} goal control request: failed to move goal to ({x}, {y})')


if __name__ == '__main__':
    socketio.start_background_task(target=send_real_time_data)
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
