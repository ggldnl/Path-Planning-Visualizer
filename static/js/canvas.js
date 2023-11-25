
export function drawLine(ctx, p1, p2, pixelOrigin, pixelScale, color, width) {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();

    // First point
    var x1_2 = pixelOrigin.x + p1[0] * pixelScale;
    var y1_2 = pixelOrigin.y - p1[1] * pixelScale;

    var x2_2 = pixelOrigin.x + p2[0] * pixelScale;
    var y2_2 = pixelOrigin.y - p2[1] * pixelScale;

    ctx.moveTo(x1_2, y1_2);
    ctx.lineTo(x2_2, y2_2);
    ctx.stroke();
    ctx.closePath();
}

function drawGridLine(ctx, x1, y1, x2, y2, color, width) {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.closePath();
}

export function drawCircle(ctx, relativePos, pixelOrigin, pixelScale, radius, lineWidth, fillColor, borderColor=null) {

    // Position of the circle in the new coordinate frame
    var x = pixelOrigin.x + relativePos[0] * pixelScale;
    var y = pixelOrigin.y - relativePos[1] * pixelScale;

    ctx.lineWidth = lineWidth;
    ctx.beginPath();
    ctx.arc(x, y, radius * pixelScale, 0, 2 * Math.PI);

    // Draw and fill the circle
    ctx.fillStyle = fillColor;
    ctx.fill();

    if (borderColor === null) {
        ctx.strokeStyle = fillColor;
    } else {
        ctx.strokeStyle = borderColor;
    }
    ctx.stroke();

    ctx.closePath();
}

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

    // Add mark on the origin
    ctx.fillStyle = textColor
    ctx.fillText(
        '0',
        originX + textOffset.x,
        originY - textOffset.y
    );

    // Draw horizontal axis
    drawGridLine(ctx, 0, originY, screenSize.width, originY, axisColor, 1);

    // Draw vertical axis
    drawGridLine(ctx, originX, 0, originX, screenSize.height, axisColor, 1);
    
    // Draw grid
    var leftEdge = Math.floor(-(screenSize.width / 2 - pixelOffset.x) / pixelScale);
    var rightEdge = Math.ceil((screenSize.width / 2 + pixelOffset.x) / pixelScale);
    for (var x = leftEdge * 10; x <= rightEdge * 10; x += 1) {
        var px = originX + pixelScale * x / 10;
        drawGridLine(ctx, px, 0, px, screenSize.height, gridColor, 0.25);

        // Draw a numeric reference
        if (pixelScale > 1000) {
            if (x !== 0 && x % 1 === 0) {
                ctx.fillStyle = textColor
                ctx.fillText(
                    (x / 10).toString() + 'm',
                    px + textOffset.x,
                    originY - textOffset.y
                );
            }
        } else if (pixelScale > 100) {
            if (x !== 0 && x % 5 === 0) {
                ctx.fillStyle = textColor
                ctx.fillText(
                    (x / 10).toString() + 'm',
                    px + textOffset.x,
                    originY - textOffset.y
                );
            }
        } else {
            if (x !== 0 && x % 10 === 0) {
                ctx.fillStyle = textColor
                ctx.fillText(
                    (x / 10).toString() + 'm',
                    px + textOffset.x,
                    originY - textOffset.y
                );
            }
        }
    }
    var topEdge = Math.floor(-(screenSize.height / 2 - pixelOffset.y) / pixelScale);
    var bottomEdge = Math.ceil((screenSize.height / 2 + pixelOffset.y) / pixelScale);
    for (var y = topEdge * 10; y <= bottomEdge * 10; y += 1) {
        var py = originY + pixelScale * y / 10;
        drawGridLine(ctx, 0, py, screenSize.width, py, gridColor, 0.25);

        // Draw a numeric reference
        if (pixelScale > 1000) {
            if (y !== 0 && y % 1 === 0) {
                ctx.fillText(
                    (-y / 10).toString() + 'm',
                    originX + textOffset.x,
                    py - textOffset.y
                );
            }
        } else if (pixelScale > 100) {
            if (y !== 0 && y % 5 === 0) {
                ctx.fillText(
                    (-y / 10).toString() + 'm',
                    originX + textOffset.x,
                    py - textOffset.y
                );
            }
        } else {
            if (y !== 0 && y % 10 === 0) {
                ctx.fillText(
                    (-y / 10).toString() + 'm',
                    originX + textOffset.x,
                    py - textOffset.y
                );
            }
        }
    }
}

export function drawPolygon(ctx, points, pixelOrigin, pixelScale, lineWidth, fillColor, borderColor=null) {

    console.log("Line width: ", lineWidth);

    if (points.length > 0) {
        ctx.lineJoin = 'round';
        ctx.lineWidth = lineWidth;
        ctx.beginPath();

        // First point
        var x0 = pixelOrigin.x + points[0][0] * pixelScale;
        var y0 = pixelOrigin.y - points[0][1] * pixelScale;

        ctx.moveTo(x0, y0);

        var x = 0;
        var y = 0;
        for (let i = 1; i < points.length; i++) {
            x = pixelOrigin.x + points[i][0] * pixelScale;
            y = pixelOrigin.y - points[i][1] * pixelScale;
            ctx.lineTo(x, y);
        }

        ctx.lineTo(x0, y0);

        ctx.fillStyle = fillColor;
        ctx.fill();

        if (borderColor === null) {
            ctx.strokeStyle = fillColor;
        } else {
            ctx.strokeStyle = borderColor;
        }
        ctx.stroke();

        ctx.closePath();
    }
}

export function drawCircumscribedPolygon(ctx, x, y, radius, lineWidth, sides, filLColor, borderColor=null) {
    ctx.lineJoin = 'round';
    ctx.lineWidth = lineWidth;
    ctx.beginPath();
    const angleStep = (2 * Math.PI) / sides;
    ctx.moveTo(x + radius * Math.cos(0), y + radius * Math.sin(0));
    for (let i = 1; i <= sides; i++) {
        const angle = i * angleStep;
        ctx.lineTo(x + radius * Math.cos(angle), y + radius * Math.sin(angle));
    }

    ctx.fillStyle = fillColor;
    ctx.fill();

    if (fillColor === null) {
        ctx.strokeStyle = borderColor;
    } else {
        ctx.strokeStyle = fillColor;
    }
    ctx.stroke();

    ctx.closePath();
}
