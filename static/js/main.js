import { drawGrid, drawLine, drawPolygon, drawCircumscribedPolygon, drawCircle } from './canvas.js';


window.onload = function () {

    // Declare the event source
    const source = new EventSource("/data");

    // Initialize canvas
    const canvas = document.getElementById("canvas");
    const ctx = canvas.getContext("2d");

    // Set width and height of the canvas to cover the entire page
    let width = canvas.width = window.innerWidth;
    let height = canvas.height = window.innerHeight;

    // Set the font
    ctx.font = "14px Roboto";

    // Scale
    let scale = 50;

    // Translation
    const pixelOffset = {
        x: 0,
        y: 0
    };

    // Text offset of the ticks text from the axis
    const textOffset = {
        x: 5,
        y: 5
    }

    const circlePosition = {
        x: 0,
        y: 0
    }

    const screenSize = {
        width: width,
        height: height
    };

    // Function to draw the canvas content
    function drawScreen() {

        // Draw the grid
        drawGrid(ctx, pixelOffset, scale, screenSize, textOffset, backgroundColor, axisColor, gridColor, textColor);

        // Draw the robot
        drawCircle(
            ctx,
            screenSize.width / 2 - pixelOffset.x + circlePosition.x,
            screenSize.height / 2 - pixelOffset.y + circlePosition.y,
            20,
            'orange',
            true
        );

        // Draw a polygon
        drawCircumscribedPolygon(
            ctx,
            screenSize.width / 2 - pixelOffset.x + circlePosition.x + 100,
            screenSize.height / 2 - pixelOffset.y + circlePosition.y + 100,
            20,
            5,
            'red',
            'round',
            true
        );

        requestAnimationFrame(drawScreen);
    }

    // Add a click event listener to the button to reset the view
    var button = document.getElementById("home");
    button.addEventListener("click", function() {
        pixelOffset.x = 0;
        pixelOffset.y = 0;
        scale = 50;
    });


    // Add a slider event listener to the obstacle speed control slider
    var obstacles_speed_slider = document.getElementById("obstacles_speed_slider");
    obstacles_speed_slider.addEventListener("input", function() {
        const obstacles_speed = obstacles_speed_slider.value;
        console.log("Updated obstacles speed value to: ", obstacles_speed);
    });


    // Resize canvas when the window is resized
    window.onresize = function () {
        width = canvas.width = window.innerWidth;
        height = canvas.height = window.innerHeight;
    };

    // Zoom in and out with mouse wheel
    canvas.onwheel = function (event) {
        const beforeOffsetX = pixelOffset.x;
        const beforeOffsetY = pixelOffset.y;
        const beforeOffsetXCart = pixelOffset.x / scale;
        const beforeOffsetYCart = pixelOffset.y / scale;
        scale -= event.deltaY * scale / 2500;
        pixelOffset.x = beforeOffsetXCart * scale;
        pixelOffset.y = beforeOffsetYCart * scale;
        if (scale < 8) {
            scale = 8;
            pixelOffset.x = beforeOffsetX;
            pixelOffset.y = beforeOffsetY;
        }
    };

    // Event handling
    source.onmessage = function (event) {

        // Get the data
        const data = JSON.parse(event.data);
        const x = data.x;
        const y = data.y;
        console.log("( " + x + ", " + y + ")");

        circlePosition.x = x;
        circlePosition.y = y;
    }

    // Mouse interaction (dragging)
    let drag = false;
    let mouseX = 0;
    let mouseY = 0;
    canvas.onmousedown = function (event) {
        drag = true;
        mouseX = event.clientX + pixelOffset.x;
        mouseY = event.clientY + pixelOffset.y;
    };
    canvas.onmousemove = function (event) {
        const currentMouseX = event.clientX;
        const currentMouseY = event.clientY;
        if (drag) {
            pixelOffset.x = mouseX - currentMouseX;
            pixelOffset.y = mouseY - currentMouseY;
        }
    };
    canvas.onmouseup = function () {
        drag = false;
    };

    /*
     * Start the canvas drawing loop. We cannot pass any argument
     * to this function since it is invoked by requestAnimationFrame
     */
    drawScreen();
};