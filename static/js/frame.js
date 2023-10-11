export class Frame {

  constructor() {
    this.drawList = [];
  }

  clear() {
    this.drawList = [];
  }

  addCircle(pos, radius, color, alpha = null) {
    this.drawList.push({
      type: "circle",
      pos: pos,
      radius: radius,
      color: color,
      alpha: alpha,
    });
  }

  addCircleFromDict(dict) {
    this.addCircle(
        dict['pos'],
        dict['radius'],
        dict['color'],
        dict['alpha']
    )
  }

  addPolygon(points, color, alpha = null) {
    this.drawList.push({
      type: "polygon",
      points: points,
      color: color,
      alpha: alpha,
    });
  }

  addPolygonFromDict(dict) {
    this.addPolygon(
        dict['points'],
        dict['color'],
        dict['alpha']
    )
  }

  addLine(line, lineWidth, color, alpha = null) {
    this.drawList.push({
      type: "line",
      lines: line,
      lineWidth: lineWidth,
      color: color,
      alpha: alpha,
    });
  }

  addLineFromDict(dict) {
    this.addLine(
        dict['line'],
        dict['lineWidth'],
        dict['color'],
        dict['alpha']
    )
  }
}
