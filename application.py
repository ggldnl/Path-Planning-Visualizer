import logging
import sys
import time
from typing import Iterator

from flask import Flask, Response, render_template, request, stream_with_context, jsonify

from scripts.frame import Frame

# Import stuff from the model
from model.world.world import World


# Configure the logger
logging.basicConfig(stream=sys.stdout, level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# Configure the Flask app
application = Flask(__name__, template_folder='template')

# Initialize a random number generator
# random.seed()

# Application refresh rate
# 20 Hz = 20 times a second: 1/20 = 0.05 update interval
REFRESH_RATE = 20  # Hz
UPDATE_FREQUENCY = 1/REFRESH_RATE

# running == True when the play button is pressed
# running == False when the stop button is pressed
running = False

# stepping == True when the step button is pressed and then
# it is set to False after one iteration
stepping = True


@application.route("/")
def index() -> str:
    return render_template("index.html")


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

        # Initialize what we will use in the control loop

        # Buffer all the geometries that will be drawn on screen
        frame = Frame()

        # Create the world
        world = World(UPDATE_FREQUENCY)

        # Dictionary containing the data to dump
        json_data = {}

        # Main loop
        global running, stepping
        while True:

            if running or stepping:

                # Clear the frame
                frame.clear()

                # Step the simulation
                world.step()

                # Add robots and obstacles to the frame
                frame.add_polygons(world.robots, 'orange')
                frame.add_polygons([obstacle.polygon for obstacle in world.map.current_obstacles], '#8B000066')

                # Add the start and the goal points to the frame
                frame.add_circle([0, 0], 0.1, 'green')
                frame.add_circle([world.map.current_goal.x, world.map.current_goal.y], 0.1, 'blue')

                # Dump the data
                json_data = frame.to_json()

                if stepping:
                    stepping = False

            yield f"data:{json_data}\n\n"
            time.sleep(UPDATE_FREQUENCY)
    except GeneratorExit:
        logger.info("Client %s disconnected", client_ip)


@application.route("/data")
def chart_data() -> Response:
    response = Response(stream_with_context(generate_data()), mimetype="text/event-stream")
    response.headers["Cache-Control"] = "no-cache"
    response.headers["X-Accel-Buffering"] = "no"
    return response


@application.route('/simulation_control', methods=['POST'])
def simulation_control():
    data = request.get_json()  # Receive the JSON data sent from the client
    print(data)

    global running, stepping
    if data and 'status' in data:
        if data['status'] == 'play':
            running = True
            response = {'status': 'Running'}
        elif data['status'] == 'stop':
            running = False
            response = {'status': 'Stopped'}
        elif data['status'] == 'step':
            stepping = True
            response = {'status': 'stepping'}
        else:
            response = {'status': 'invalid'}

        return jsonify(response)
    else:
        return jsonify({'status': 'Invalid data or value received.'}), 400


if __name__ == "__main__":
    application.run(host="0.0.0.0", threaded=True)

