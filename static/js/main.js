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
    scale = 50;

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

    // Function to draw the canvas content
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
        }

        function drawCircle() {
            ctx.beginPath();
            ctx.arc(pixelOrigin.x + circlePosition.x, pixelOrigin.y + circlePosition.y, 20, 0, 2 * Math.PI);
            ctx.fillStyle = 'orange';
            ctx.fill();
            ctx.closePath();
        }

        function drawTestLine() {
            // Draw a line on the screen
            ctx.strokeStyle = 'red';
            ctx.beginPath();
            ctx.moveTo(pixelOrigin.x, pixelOrigin.y);
            ctx.lineTo(150, 150);
            ctx.lineWidth = 0.25;
            ctx.stroke();
        }

        function drawVerticalAxis() {
            ctx.beginPath();
            ctx.moveTo(pixelOrigin.x, 0);
            ctx.lineTo(pixelOrigin.x, height);
            ctx.strokeStyle = axisColor;
            ctx.lineWidth = 1;
            ctx.stroke();
        }

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
                if (x !== 0 && x % 5 === 0) {
                    ctx.fillText(
                        x.toString(),
                        px + textOffset.x,
                        pixelOrigin.y - textOffset.y
                    );
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
                if (y !== 0 && y % 5 === 0) {
                    ctx.fillText(
                        (-y).toString(),
                        pixelOrigin.x + textOffset.x,
                        py - textOffset.y
                    );
                }
            }
        }

        drawHorizontalAxis();
        drawVerticalAxis();
        drawGrid();
        drawCircle();
        requestAnimationFrame(drawScreen);
    }

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