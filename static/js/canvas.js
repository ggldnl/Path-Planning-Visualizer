import { backgroundColor, axisColor, gridColor, fontColor, font } from './style.js';

var socket = io.connect('http://' + document.domain + ':' + location.port);

// Data variable that will contain the shapes to show
var data;

socket.on('real_time_data', function (string_data) {
    const shapes = JSON.parse(string_data);
    data = shapes;
});


window.onload = function () {

    var canvas = document.getElementById("canvas");
    var home_btn = document.getElementById("home-btn");
    var ctx = canvas.getContext("2d");

    var width = canvas.width = window.innerWidth;
    var height = canvas.height = window.innerHeight;

    var pixelOffset = {
        x: 0,
        y: 0
    };
    var scale = 50;
    const initial_scale = scale;

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

            // Text offset of the ticks text from the axis
            const textDisplacement = {
                x: 5,
                y: 5
            }

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

        drawHorizontalAxis();
        drawVerticalAxis();
        drawGrid();

        if (data !== undefined) {
            for (var element of data) {

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
                }
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
        if (scale < 10) {
            scale = 10;
            pixelOffset.x = beforeOffsetX;
            pixelOffset.y = beforeOffsetY;
        }
        if (scale > 200) {
            scale = 200;
            pixelOffset.x = beforeOffsetX;
            pixelOffset.y = beforeOffsetY;
        }
    };

    {
        var dragging = false;
        var mouseX_new = 0;
        var mouseY_new = 0;

        canvas.onmousedown = function (event) {
            dragging = true;
            mouseX_new = event.clientX + pixelOffset.x;
            mouseY_new = event.clientY + pixelOffset.y;
        };

        canvas.onmousemove = function (event) {
            var currentMouseX = event.clientX;
            var currentMouseY = event.clientY;
            if (dragging) {
                pixelOffset.x = mouseX_new - currentMouseX;
                pixelOffset.y = mouseY_new - currentMouseY;
            }
        };

        canvas.onmouseup = function (event) {
            dragging = false;
        };
    }
    drawScreen();
};
