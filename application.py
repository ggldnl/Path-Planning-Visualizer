import json
import logging
import sys
import time
from datetime import datetime
from typing import Iterator

from flask import Flask, Response, render_template, request, stream_with_context

from model.test.ball import Ball


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

# Create an object to move on the screen
ball_radius = 20
bounding_box_start_x, bounding_box_end_x = -250, 250
bounding_box_start_y, bounding_box_end_y = -250, 250
ball = Ball(
    ball_radius,
    bounding_box_start_x,
    bounding_box_end_x,
    bounding_box_start_y,
    bounding_box_end_y
)


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

        # Main loop
        while True:

            ball.step(UPDATE_FREQUENCY)

            # Dump the data
            json_data = json.dumps(
                {
                    "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "x": ball.x,
                    "y": ball.y
                }
            )
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


if __name__ == "__main__":
    application.run(host="0.0.0.0", threaded=True)
