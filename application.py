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

import logging
import sys
import time
from typing import Iterator
import numpy as np

# Flask imports
from flask import Flask, Response, render_template, request, stream_with_context, jsonify

from model.exceptions.collision_exception import CollisionException
from model.geometry.point import Point
from model.world.controllers.a_r_controller import AStarController
# Import scripts
from scripts.frame import Frame

# Import stuff from the model
from model.world.world import World
from model.world.color_palette import *
from model.world.robot.URDF_parser import URDFParser
from model.world.robot.differential_drive_robot import DifferentialDriveRobot
from model.world.robot.robots.cobalt.cobalt import Cobalt
from model.world.controllers.a_star_controller import AStarController

# Configure the logger
logging.basicConfig(stream=sys.stdout, level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# Configure the Flask app
application = Flask(__name__, template_folder='template')

# Initialize a random number generator
# random.seed()

# ---------------------------- defining some stuff --------------------------- #

# Application refresh rate
# 20 Hz = 20 times a second: 1/20 = 0.05 update interval
REFRESH_RATE = 20  # Hz
UPDATE_FREQUENCY = 1 / REFRESH_RATE

# running == True when the play button is pressed
# running == False when the stop button is pressed
running = False

# stepping == True when the step button is pressed and then
# it is set to False after one iteration
stepping = True

# Boolean controlling whether to show the respective elements
show_trace = False
show_sensors = False
show_path = False

# TODO read this!
# Boolean to update only the robot (some of its attributes needs to be shown or hidden).
# It will be turned off the iteration after the update happens.
#
# This can be solved by simply updating the frame each time regardless of the value of
# running and stepping variables. This poses another problem: with the actual communication
# protocol between frontend and backend, the amount of data exchanged is very big, so by
# not sending data when the simulation is stopped we can reduce the memory consumption.
# This should be fixed in later updates!
update_robot = False

# Define the world here so we can access it through the routes
world = World(UPDATE_FREQUENCY)

# Buffer for all the geometries that will be drawn on screen
frame = Frame()

# ------------------------------ generation loop ----------------------------- #


def generate_data() -> Iterator[str]:
    """
    This section checks if the HTTP request headers contain an "X-Forwarded-For" header.
    This header is commonly used to store the original IP address of the client
    when requests pass through one or more proxy servers or load balancers.
    """
    if request.headers.getlist("X-Forwarded-For"):
        """
        If the "X-Forwarded-For" header is present in the request headers 
        (i.e., request.headers.getlist("X-Forwarded-For") is not empty), 
        it extracts the client's IP address from the first element of the list.
        """
        client_ip = request.headers.getlist("X-Forwarded-For")[0]
    else:
        """
        If the "X-Forwarded-For" header is not present or empty, it falls back to 
        using request.remote_addr. request.remote_addr is Flask's way of accessing 
        the IP address of the client making the request. 
        If this is also unavailable, it sets client_ip to an empty string ("").
        """
        client_ip = request.remote_addr or ""

    try:

        logger.info("Client %s connected", client_ip)

        # Dictionary containing the data to dump
        json_data = {}

        # Main loop
        global running, stepping
        # running = False
        # stepping = True
        while True:

            if running or stepping:

                try:

                    # Step the simulation
                    world.step()

                    # Clear the frame
                    frame.clear()

                    # Add the robot to the frame
                    for robot, controller in zip(world.robots, world.controllers):

                        if show_path:
                            if controller.path:

                                points = [[x, y] for x, y in controller.path]

                                # robot_x, robot_y, _ = robot.current_pose
                                # frame.add_line([robot_x, robot_y], points[-1], 1, '#ff0000')

                                for i in range(1, len(points)):
                                    frame.add_line(points[i - 1], points[i], 1, path_color)

                        frame.add_polygons(robot.bodies, default_robot_fill_color, default_robot_border_color)

                        # Add sensors if the option is enabled
                        if show_sensors:
                            frame.add_polygons([sensor.polygon for sensor in robot.sensors], sensor_color_alert_0)

                        """
                        # Add the path
                        if show_path:
                            frame.add_lines()
                    
                        # Add the controller path
                        # Boh
                        """

                    # Add the obstacles to the frame (we can change color for moving and steady obstacles)

                    # steady_obstacles = [
                    # obstacle.polygon for obstacle in world.map_legacy.current_obstacles if obstacle.vel == (0, 0, 0)
                    # ]
                    # moving_obstacles = [
                    # obstacle.polygon for obstacle in world.map_legacy.current_obstacles if obstacle.vel != (0, 0, 0)
                    # ]
                    # frame.add_polygons(steady_obstacles, '#8B000066')
                    # frame.add_polygons(moving_obstacles, '#8B000066')

                    frame.add_polygons([obstacle.polygon for obstacle in world.map.obstacles], obstacle_fill_color)

                    # Add the start and the goal points to the frame
                    frame.add_circle([world.map.current_goal.x, world.map.current_goal.y], 0.025, goal_fill_color)

                    # Dump the data
                    json_data = frame.to_json()

                    if stepping:
                        stepping = False

                    yield f"data:{json_data}\n\n"
                    time.sleep(UPDATE_FREQUENCY)

                    # Check for collisions; if the case, stop
                    world.apply_physics()

                    # world.search()

                except CollisionException:
                    running = False
                    break

    except GeneratorExit:
        logger.info("Client %s disconnected", client_ip)


# ------------------------------- flask routes ------------------------------- #


@application.route("/")
def index() -> str:
    return render_template("index.html")


@application.route("/data")
def chart_data() -> Response:
    response = Response(stream_with_context(generate_data()), mimetype="text/event-stream")
    response.headers["Cache-Control"] = "no-cache"
    response.headers["X-Accel-Buffering"] = "no"
    return response


@application.route('/simulation_control', methods=['POST'])
def simulation_control():
    data = request.get_json()  # Receive the JSON data sent from the client

    # Reference the global boolean control variables
    global running, stepping
    global show_trace, show_sensors, show_path
    global update_robot

    if data:

        if 'status' in data:
            if data['status'] == 'play':
                running = True
            elif data['status'] == 'stop':
                running = False
            elif data['status'] == 'step':
                stepping = True
            elif data['status'] == 'reset':
                running = False
                stepping = True
                world.map.reset_map()

        if 'obs_lin_speed' in data:
            speed_multiplier = float(data['obs_lin_speed'])
            for obstacle in world.map.obstacles:
                obstacle.linear_speed_multiplier = speed_multiplier

        if 'obs_ang_speed' in data:
            speed_multiplier = float(data['obs_ang_speed'])
            for obstacle in world.map.obstacles:
                obstacle.angular_speed_multiplier = speed_multiplier

        if 'robot_linear_velocity' in data:
            linear_velocity = float(data['robot_linear_velocity'])
            for robot in world.robots:
                robot.linear_velocity = linear_velocity

        if 'robot_angular_velocity' in data:
            angular_velocity = float(data['robot_angular_velocity'])
            for robot in world.robots:
                robot.angular_velocity = angular_velocity

        if 'random_map' in data:
            world.map.get_map(world.robots)
            running = False
            stepping = True

        if 'load' in data:
            file_content = data['load']
            world.map.load_map_from_json_data(file_content)
            world.reset_robots()
            running = False
            stepping = True

        if 'save' in data:
            file_path = data['save']
            world.map.save_map(file_path)

        if 'show' in data:
            flag = data['show']
            if flag == 'trace':
                show_trace = not show_trace
            if flag == 'sensors':
                show_sensors = not show_sensors
            if flag == 'path':
                show_path = not show_path

            # TODO update the robot (cascading) when one of these checkboxes is ticked
            update_robot = True

        if 'direction' in data:

            # TODO for now, only the first robot can be controlled with the arrow keys.
            # Future implementations can use the mouse to select one of the robots
            dir = data['direction']
            robot = world.robots[0]

            step_size = 0.1  # m
            x, y, theta = robot.current_pose

            if dir == 'up':

                new_x = x + step_size * np.cos(theta)
                new_y = y + step_size * np.sin(theta)
                delta_x = new_x - x
                delta_y = new_y - y
                new_theta = np.arctan2(delta_y, delta_x)

                robot.target_pose = (new_x, new_y, new_theta)

            elif dir == 'down':

                # This is not feasible since part of the algorithm to step the robot
                # turns the robot towards the point to reach. In a backward movement
                # the orientation remains unchanged
                #
                # new_x = x - step_size * np.cos(theta)
                # new_y = y - step_size * np.sin(theta)
                # robot.target_pose = (new_x, new_y, -theta)
                pass

            elif dir == 'left':

                new_theta = theta + step_size
                new_theta = new_theta % (2 * np.pi)
                robot.target_pose = (x, y, new_theta)

            elif dir == 'right':

                new_theta = theta - step_size
                new_theta = new_theta % (2 * np.pi)
                robot.target_pose = (x, y, new_theta)

            print(f'Received [{dir}]: new target pose: {world.robots[0].target_pose}')

        response = {'status': 'Changes registered'}
        return jsonify(response)

    else:
        return jsonify({'status': 'Invalid data or value received.'}), 400


if __name__ == "__main__":
    # Create the robot

    # robot_polygons = URDFParser.parse('./model/world/robot/robots/R2D2/R2D2.urdf')
    # robot = DifferentialDriveRobot(robot_polygons)
    # controller = None
    # world.add_robot(robot, controller)
    robot = Cobalt()
    controller = AStarController(world.map.goal, robot)
    world.add_robot(robot, controller)
    application.run(host="0.0.0.0", port=5000, threaded=True)
