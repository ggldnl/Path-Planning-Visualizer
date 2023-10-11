
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
export function drawCircle(ctx, relativePos, pixelOrigin, pixelScale, radius, color, filled) {

    // Position of the circle in the new coordinate frame
    var x = pixelOrigin.x + relativePos[0] * pixelScale;
    var y = pixelOrigin.y - relativePos[1] * pixelScale;

    ctx.beginPath();
    ctx.arc(x, y, radius * pixelScale, 0, 2 * Math.PI);
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
 * @param {string} textColor - The color of the text.
 */
export function drawGrid(
        ctx,
        pixelOffset,
        pixelScale,
        screenSize,
        textOffset,
        backgroundColor,
        axisColor,
        gridColor,
        textClor
    ) {

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
        var px = originX + pixelScale * x;
        drawLine(ctx, px, 0, px, screenSize.height, gridColor, 0.25);
        
        // Draw a numeric reference
        if (x !== 0 && x % 5 === 0) {
            ctx.fillStyle = textColor
            ctx.fillText(
                x.toString(),
                px + textOffset.x,
                originY - textOffset.y
            );
        }
    }
    var topEdge = Math.floor(-(screenSize.height / 2 - pixelOffset.y) / pixelScale);
    var bottomEdge = Math.ceil((screenSize.height / 2 + pixelOffset.y) / pixelScale);
    for (var y = topEdge; y <= bottomEdge; y++) {
        var py = originY + pixelScale * y;
        drawLine(ctx, 0, py, screenSize.width, py, gridColor, 0.25);

        // Draw a numeric reference
        if (y !== 0 && y % 5 === 0) {
            ctx.fillText(
                (-y).toString(),
                originX + textOffset.x,
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
 * @param {string} lineJoin - Line join type (e.g. 'round' to smooth the edges of the polygon).
 * @param {boolean} filled - A boolean value indicating whether the polygon should be filled or only outlined.
 */
export function drawPolygon(ctx, points, pixelOrigin, pixelScale, color, lineJoin, filled) {
    if (points.length > 0) {
        ctx.lineJoin = lineJoin;
        ctx.beginPath();

        // First point
        var x = pixelOrigin.x + points[0][0] * pixelScale;
        var y = pixelOrigin.y - points[0][1] * pixelScale;

        ctx.moveTo(x, y);

        for (let i = 1; i < points.length; i++) {
            x = pixelOrigin.x + points[i][0] * pixelScale;
            y = pixelOrigin.y - points[i][1] * pixelScale;
            ctx.lineTo(x, y);
        }

        ctx.closePath();
        if (filled) {
            ctx.fillStyle = color;
            ctx.fill();
        } else {
            ctx.strokeStyle = color;
            ctx.stroke();
        }
    }
}

/**
 * Draws a polygon inscribed into the specified circumference on the canvas.
 *
 * @param {CanvasRenderingContext2D} ctx - The canvas 2D rendering context on which to draw the polygon.
 * @param {number} x - The x-coordinate of the center of the polygon.
 * @param {number} y - The y-coordinate of the center of the polygon.
 * @param {number} radius - The radius of the circumscribed circle of the polygon.
 * @param {number} sides - The number of sides of the polygon (e.g., 3 for a triangle, 5 for a pentagon, etc.).
 * @param {string} color - The color of the polygon's outline or fill (e.g., 'green', '#FFA500', 'rgba(255, 0, 0, 0.8)').
 * @param {string} lineJoin - Line join type (e.g. 'round' to smooth the edges of the polygon).
 * @param {boolean} filled - A boolean value indicating whether the polygon should be filled (`true`) or only outlined (`false`).
 */
export function drawCircumscribedPolygon(ctx, x, y, radius, sides, color, lineJoin, filled) {
    ctx.lineJoin = lineJoin;
    ctx.beginPath();
    const angleStep = (2 * Math.PI) / sides;
    ctx.moveTo(x + radius * Math.cos(0), y + radius * Math.sin(0));
    for (let i = 1; i <= sides; i++) {
        const angle = i * angleStep;
        ctx.lineTo(x + radius * Math.cos(angle), y + radius * Math.sin(angle));
    }
    ctx.closePath();
    if (filled) {
        ctx.fillStyle = color;
        ctx.fill();
    } else {
        ctx.strokeStyle = color;
        ctx.stroke();
    }
}
