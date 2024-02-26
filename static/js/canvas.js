import { backgroundColor, axisColor, gridColor, fontColor, font } from './style.js';
import { socket } from './websocket.js';

// var socket = io.connect('http://' + document.domain + ':' + location.port);

// Data variable that will contain the shapes to show
var json_data;
var algorithm;
var iterationsPerStep;
var data;

// Double click
var clickCount = 0;
var clickTimer;

socket.on('real_time_data', function (string_data) {
    json_data = string_data;
    data = JSON.parse(string_data);
    // console.log(data);

    var controllers = data['controllers'];
    controllers.forEach(function (controller) {

        var searchAlgorithm = controller['search_algorithm'];

        // Update currently selected search algorithm
        var searchAlgorithmClass = searchAlgorithm["class"];
        if (searchAlgorithmClass != algorithm) {
            algorithm = searchAlgorithmClass;
            updateCurrentAlgorithmButton(searchAlgorithmClass);
        }

        // Update iterations_per_step slider
        var searchAlgorithmIterationsPerStep = searchAlgorithm['iterations_per_step']
        if (searchAlgorithmIterationsPerStep != iterationsPerStep) {
            iterationsPerStep = searchAlgorithmIterationsPerStep;
            updateIterationsPerStepSlider(iterationsPerStep);
        }

        // Update the progress bar
        var currentIteration = searchAlgorithm["current_iteration"];
        var maxIteration = searchAlgorithm["max_iterations"];
        updateProgressBar(currentIteration, maxIteration);

    });

    // console.log('Algorithm:', data['controllers'][0])
});

function updateCurrentAlgorithmButton(algorithm) {

    var buttons = document.querySelectorAll('.radio-button');
    var foundButton = null;

    buttons.forEach(function (button) {

        // Remove the checked class
        button.classList.remove('checked');

        // Get the text content of the button and remove spaces
        var buttonTextWithoutSpaces = button.textContent.replace(/\s/g, '');

        // Check if the text without spaces matches the provided text
        if (buttonTextWithoutSpaces === algorithm) {
            foundButton = button; // Set the matching button
        }
    });

    foundButton.classList.add('checked');
}

function updateIterationsPerStepSlider(iterationsPerStep) {

    var slider = document.getElementById('iterations-per-step-slider');
    slider.value = iterationsPerStep;

}

function updateProgressBar(currentIteration, maxIteration) {

    var progressBar = document.getElementById('progress-bar');

    var percentage = (currentIteration / maxIteration) * 100;

    progressBar.style.width = percentage + '%';
    progressBar.textContent = percentage.toFixed(2) + '%';

}

// Magic tool we will need later (strumentopolo misterioso)
const track_btn = document.getElementById('track-btn');

document.getElementById('save-btn').addEventListener('click', function() {

    // Create a Blob with the JSON data
    const blob = new Blob([json_data], { type: 'application/json' });

    // Create a link element
    var link = document.createElement("a");

    // Set the download attribute with the desired filename
    link.download = "world.json";

    // Create a URL for the Blob and set it as the href attribute
    link.href = window.URL.createObjectURL(blob);

    // Append the link to the body
    document.body.appendChild(link);

    // Trigger a click event on the link to start the download
    link.click();

    // Remove the link from the DOM
    document.body.removeChild(link);

});

// Whether to show or not obstacle IDs
var show_obstacle_ids = true;
document.getElementById('show-obstacle-ids-chk').addEventListener('change', function() {
    show_obstacle_ids = this.checked;
});

window.onload = function () {

    var canvas = document.getElementById("canvas");
    var home_btn = document.getElementById("home-btn");
    var ctx = canvas.getContext("2d");

    var width = canvas.width = window.innerWidth;
    var height = canvas.height = window.innerHeight;

    // Text offset from a point
    const textDisplacement = {
        x: 5,
        y: 5
    }

    var pixelOffset = {
        x: 0,
        y: 0
    };
    var scale = 350;
    const min_scale = 10;
    const max_scale = 500;
    const initial_scale = scale;

    track_btn.onclick = function (event) {
        // TODO provide multi robot native support
        // Robot coordinates
        var pose = data['robots'][0]['pose']
        var robot_coords = {
            x: pose['x'],
            y: pose['y']
        }
        console.log(robot_coords)

        pixelOffset.x = robot_coords.x * scale;
        pixelOffset.y = -robot_coords.y * scale;
        scale = max_scale;
    }

    function drawScreen() {

        ctx.clearRect(0, 0, width, height);
        ctx.fillStyle = backgroundColor;
        ctx.strokeStyle = backgroundColor;
        ctx.fillRect(0, 0, width, height);

        var pixelOrigin = {
            x: width / 2 - pixelOffset.x,
            y: height / 2 - pixelOffset.y
        };

        function drawHorizontalAxis() {
            ctx.beginPath();
            ctx.moveTo(0, pixelOrigin.y);
            ctx.lineTo(width, pixelOrigin.y);
            ctx.strokeStyle = axisColor;
            ctx.lineWidth = 1;
            ctx.stroke();
        };

        function drawVerticalAxis() {
            ctx.beginPath();
            ctx.moveTo(pixelOrigin.x, 0);
            ctx.lineTo(pixelOrigin.x, height);
            ctx.strokeStyle = axisColor;
            ctx.lineWidth = 1;
            ctx.stroke();
        };

        function drawGrid() {

            ctx.strokeStyle = gridColor;
            ctx.fillStyle = fontColor;
            var leftEdge = Math.floor(-(width / 2 - pixelOffset.x) / scale);
            var rightEdge = Math.ceil((width / 2 + pixelOffset.x) / scale);

            for (var x = leftEdge; x <= rightEdge; x++) {
                var px = pixelOrigin.x + scale * x;
                ctx.beginPath();
                ctx.moveTo(px, 0);
                ctx.lineTo(px, height);
                ctx.lineWidth = 0.25;
                ctx.stroke();

                if (scale > 70) {
                    if (x % 1 === 0) {
                        ctx.fillText(
                            (x / 10).toString() + 'm',
                            px + textDisplacement.x,
                            pixelOrigin.y - textDisplacement.y
                        );
                    }
                } else if (scale > 30) {
                    if (x % 5 === 0) {
                        ctx.fillText(
                            (x / 10).toString() + 'm',
                            px + textDisplacement.x,
                            pixelOrigin.y - textDisplacement.y
                        );
                    }
                } else {
                    if (x % 10 === 0) {
                        ctx.fillText(
                            (x / 10).toString() + 'm',
                            px + textDisplacement.x,
                            pixelOrigin.y - textDisplacement.y
                        );
                    }
                }
            }

            var topEdge = Math.floor(-(height / 2 - pixelOffset.y) / scale);
            var bottomEdge = Math.ceil((height / 2 + pixelOffset.y) / scale);
            for (var y = topEdge; y <= bottomEdge; y++) {
                var py = pixelOrigin.y + scale * y;
                ctx.beginPath();
                ctx.moveTo(0, py);
                ctx.lineTo(width, py);
                ctx.lineWidth = 0.25;
                ctx.stroke();

                if (scale > 70) {
                    if (y % 1 === 0) {
                        ctx.fillText(
                            (-y / 10).toString() + 'm',
                            pixelOrigin.x + textDisplacement.x,
                            py - textDisplacement.y
                        );
                    }
                } else if (scale > 30) {
                    if (y % 5 === 0) {
                        ctx.fillText(
                            (-y / 10).toString() + 'm',
                            pixelOrigin.x + textDisplacement.x,
                            py - textDisplacement.y
                        );
                    }
                } else {
                    if (y % 10 === 0) {
                        ctx.fillText(
                            (-y / 10).toString() + 'm',
                            pixelOrigin.x + textDisplacement.x,
                            py - textDisplacement.y
                        );
                    }
                }

            }
        };

        function drawPolygon(points, fillColor, borderColor, lineWidth) {

            ctx.lineJoin = 'round';
            ctx.lineWidth = lineWidth;

            // First point
            ctx.beginPath();
            ctx.moveTo(pixelOrigin.x + points[0][0] * scale, pixelOrigin.y - points[0][1] * scale);

            // Add the other points
            for (let i = 1; i < points.length; i++) {
                ctx.lineTo(pixelOrigin.x + points[i][0] * scale, pixelOrigin.y - points[i][1] * scale);
            }

            // Close the path
            ctx.closePath();

            // Fill the path
            ctx.fillStyle = fillColor;
            ctx.fill();

            // Stroke the path
            if (borderColor === null) {
                ctx.strokeStyle = fillColor;
            } else {
                ctx.strokeStyle = borderColor;
            }
            ctx.stroke();

        }

        function drawCircle(center, radius, fillColor, borderColor, lineWidth) {

            ctx.lineWidth = lineWidth;

            ctx.beginPath();
            var x = pixelOrigin.x + center[0] * scale;
            var y = pixelOrigin.y - center[1] * scale;
            ctx.arc(x, y, radius * scale, 0, 2 * Math.PI);
            ctx.closePath();

            // Draw and fill the circle
            ctx.fillStyle = fillColor;
            ctx.fill();

            if (borderColor === null) {
                ctx.strokeStyle = fillColor;
            } else {
                ctx.strokeStyle = borderColor;
            }
            ctx.stroke();

        }

        function drawSegment(p1, p2, color, lineWidth) {

            ctx.strokeStyle = color;
            ctx.lineWidth = lineWidth;

            // First point
            ctx.beginPath();
            ctx.moveTo(pixelOrigin.x + p1[0] * scale, pixelOrigin.y - p1[1] * scale);

            // Second point
            ctx.lineTo(pixelOrigin.x + p2[0] * scale, pixelOrigin.y - p2[1] * scale);
            ctx.closePath();

            ctx.stroke();
        }

        function drawEllipse(centerX, centerY, a, b, phi, fillColor, borderColor, lineWidth) {
            ctx.lineWidth = lineWidth;

            // Translate and rotate the context to draw the rotated ellipse
            ctx.translate(pixelOrigin.x + centerX * scale, pixelOrigin.y - centerY * scale);
            ctx.rotate(phi);

            // Draw the ellipse
            ctx.beginPath();
            ctx.ellipse(0, 0, a * scale, b * scale, 0, 0, 2 * Math.PI);
            ctx.closePath();

            // Fill the ellipse
            ctx.fillStyle = fillColor;
            ctx.fill();

            // Stroke the ellipse
            if (borderColor === null) {
                ctx.strokeStyle = fillColor;
            } else {
                ctx.strokeStyle = borderColor;
            }
            ctx.stroke();

            // Reset the transformation
            ctx.setTransform(1, 0, 0, 1, 0, 0);
        }


        drawHorizontalAxis();
        drawVerticalAxis();
        drawGrid();

        if (data !== undefined) {

            data["view"].forEach(element => {

                /*
                // Iterate over key-value pairs and print them
                Object.entries(element).forEach(([key, value]) => {
                    console.log(`${key}: ${value}`);
                });
                */

                var type = element["type"];
                if (type === "polygon") {
                    drawPolygon(element["points"], element["fill_color"], element["border_color"], element["line_width"]);
                } else if (type == "circle") {
                    drawCircle(element["center"], element["radius"], element["fill_color"], element["border_color"], element["line_width"]);
                } else if (type == "segment") {
                    drawSegment(element["p1"], element["p2"], element["color"], element["line_width"]);
                } else if (type == "ellipse") {
                    drawEllipse(element["f1"], element["f2"], element["c"], element["fill_color"], element["border_color", element["line_width"]]);
                }
            })

            if (show_obstacle_ids) {

                // Set the text color
                ctx.fillStyle = fontColor;

                data["map"]["obstacles"].forEach(obstacle_dict => {
                    var x = obstacle_dict["obstacle"]["polygon"]["pose"]["x"];
                    var y = obstacle_dict["obstacle"]["polygon"]["pose"]["y"];
                    var id = obstacle_dict["id"];
                    var px = pixelOrigin.x + scale * x;
                    var py = pixelOrigin.y - scale * y;
                    ctx.fillText(id, px, py);
                })
            }
        }

        requestAnimationFrame(drawScreen);
    };

    window.onresize = function () {
        width = canvas.width = window.innerWidth;
        height = canvas.height = window.innerHeight;
    };

    // Add a click event listener to the button to reset the view
    home_btn.addEventListener("click", function() {
        pixelOffset.x = 0;
        pixelOffset.y = 0;
        scale = initial_scale;
    });

    canvas.onwheel = function (event) {
        var beforeOffsetX = pixelOffset.x;
        var beforeOffsetY = pixelOffset.y;
        var beforeOffsetXCart = pixelOffset.x / scale;
        var beforeOffsetYCart = pixelOffset.y / scale;
        scale -= event.deltaY * scale / 2500;
        pixelOffset.x = beforeOffsetXCart * scale;
        pixelOffset.y = beforeOffsetYCart * scale;
        if (scale < min_scale) {
            scale = min_scale;
            pixelOffset.x = beforeOffsetX;
            pixelOffset.y = beforeOffsetY;
        }
        if (scale > max_scale) {
            scale = max_scale;
            pixelOffset.x = beforeOffsetX;
            pixelOffset.y = beforeOffsetY;
        }
    };

    {
        var mouseDown = false;
        var clicking = false;
        var mouseX_new = 0;
        var mouseY_new = 0;

        canvas.onmousedown = function (event) {
            mouseDown = true;
            clicking = true;
            mouseX_new = event.clientX + pixelOffset.x;
            mouseY_new = event.clientY + pixelOffset.y;
        };

        canvas.onmousemove = function (event) {
            var currentMouseX = event.clientX;
            var currentMouseY = event.clientY;
            if (mouseDown) {
                clicking = false; // User is dragging
                pixelOffset.x = mouseX_new - currentMouseX;
                pixelOffset.y = mouseY_new - currentMouseY;
            }
        };

        canvas.onmouseup = function (event) {
            mouseDown = false;
            if (clicking) {

                // Change coordinate system
                var x = (event.clientX - width / 2 + pixelOffset.x) / scale;
                var y = - (event.clientY - height / 2 + pixelOffset.y) / scale;

                clickCount ++;
                if (clickCount == 1) {
                    clickTimer = setTimeout(function() {

                        // Send obstacle position to backend
                        console.log('Add obstacle at (', x, ', ', y, ')')
                        socket.emit('obstacle_control', x, y);

                        clickCount = 0;
                    }, 250);
                } else if (clickCount == 2) {
                    clearTimeout(clickTimer);

                    // Send goal position to backend
                    console.log('Move goal to (', x, ', ', y, ')')
                    socket.emit('goal_control', x, y);

                    clickCount = 0;
                }

                clicking = false;
            }
        };
    }
    drawScreen();
};
