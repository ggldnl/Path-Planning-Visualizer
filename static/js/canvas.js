
/**
 * Draws a straight line on the canvas context ctx from the point (x1, y1)
 * to the point (x2, y2) with the specified line color and width.
 *
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context on which to draw the line.
 * @param {number} x1 - The x-coordinate of the starting point of the line.
 * @param {number} y1 - The y-coordinate of the starting point of the line.
 * @param {number} x2 - The x-coordinate of the ending point of the line.
 * @param {number} y2 - The y-coordinate of the ending point of the line.
 * @param {string} color - The color of the line (e.g., 'red', '#00FF00', 'rgba(255, 0, 0, 0.5)').
 * @param {number} width - The width of the line in pixels.
 */
export function drawLine(ctx, x1, y1, x2, y2, color, width) {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
}

/**
 * Draws a circle with radius r on the canvas in (x,y).
 *
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context on which to draw the circle.
 * @param {number} x - The x-coordinate of the center of the circle.
 * @param {number} y - The y-coordinate of the center of the circle.
 * @param {number} radius - The radius of the circle in pixels.
 * @param {string} color - The color of the circle's outline or fill (e.g., 'blue', '#FFA500', 'rgba(0, 0, 255, 0.8)').
 * @param {boolean} filled - A boolean value indicating whether the circle should be filled (`true`) or only outlined (`false`).
 */
export function drawCircle(ctx, x, y, radius, color, filled) {
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, 2 * Math.PI);
    if (filled) {
        ctx.fillStyle = color;
        ctx.fill();
    } else {
        ctx.strokeStyle = color;
        ctx.stroke();
    }
    ctx.closePath();
}

/**
 * Draws a cartesian grid on the canvas with the specified properties.
 *
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context on which to draw.
 * @param {Object} pixelOffset - An object containing x and y pixel offsets (translation).
 * @param {number} pixelScale - The scale factor for mapping pixels to coordinates.
 * @param {Object} screenSize - An object containing the width and height of the canvas/screen.
 * @param {Object} textOffset - An object containing x and y offsets of text elements wrt the axis.
 * @param {string} backgroundColor - The background color of the canvas.
 * @param {string} axisColor - The color of the horizontal and vertical axes.
 * @param {string} gridColor - The color of the grid lines.
 */
export function drawScreen(ctx, pixelOffset, pixelScale, screenSize, textOffset, backgroundColor, axisColor, gridColor) {

    ctx.clearRect(0, 0, screenSize.width, screenSize.height);
    ctx.fillStyle = backgroundColor;
    ctx.strokeStyle = backgroundColor;
    ctx.fillRect(0, 0, screenSize.width, screenSize.height);

    var originX = screenSize.width / 2 - pixelOffset.x;
    var originY = screenSize.height / 2 - pixelOffset.y;

    // Draw horizontal axis
    drawLine(ctx, 0, originY, screenSize.width, originY, axisColor, 1);

    // Draw vertical axis
    drawLine(ctx, originX, 0, originX, screenSize.height, axisColor, 1);
    
    // Draw grid
    var leftEdge = Math.floor(-(screenSize.width / 2 - pixelOffset.x) / pixelScale);
    var rightEdge = Math.ceil((screenSize.width / 2 + pixelOffset.x) / pixelScale);
    for (var x = leftEdge; x <= rightEdge; x++) {
        var px = pixelOrigin.x + pixelScale * x;
        drawLine(ctx, px, 0, px, screenSize.height, gridColor, 0.25);
        
        // Draw a numeric reference
        if (x !== 0 && x % 5 === 0) {
            ctx.fillText(
                x.toString(),
                px + textOffset.x,
                pixelOrigin.y - textOffset.y
            );
        }
    }
    var topEdge = Math.floor(-(screenSize.height / 2 - pixelOffset.y) / pixelScale);
    var bottomEdge = Math.ceil((screenSize.height / 2 + pixelOffset.y) / pixelScale);
    for (var y = topEdge; y <= bottomEdge; y++) {
        var py = pixelOrigin.y + pixelScale * y;
        drawLine(ctx, 0, py, screenSize.width, py, gridColor, 0.25);

        // Draw a numeric reference
        if (y !== 0 && y % 5 === 0) {
            ctx.fillText(
                (-y).toString(),
                pixelOrigin.x + textOffset.x,
                py - textOffset.y
            );
        }
    }
}

/**
 * Draws a polygon on the canvas context.
 *
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context on which to draw the polygon.
 * @param {Array} points - An array of objects representing the points of the polygon, each having x and y coordinates.
 * @param {string} color - The color of the polygon's outline or fill (e.g., 'green', '#FFA500', 'rgba(255, 0, 0, 0.8)').
 * @param {boolean} filled - A boolean value indicating whether the polygon should be filled or only outlined.
 */
export function drawPolygon(ctx, points, color, filled) {
    if (points.length > 0) {
        ctx.beginPath();
        ctx.moveTo(points[0].x, points[0].y);
        for (let i = 1; i < points.length; i++) {
            ctx.lineTo(points[i].x, points[i].y);
        }
        if (filled) {
            ctx.fillStyle = color;
            ctx.fill();
        } else {
            ctx.strokeStyle = color;
            ctx.stroke();
        }
        ctx.closePath();
    }
}
