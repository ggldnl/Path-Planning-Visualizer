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

  addPolygon(polygon, color, alpha = null) {
    this.drawList.push({
      type: "polygon",
      polygons: polygon,
      color: color,
      alpha: alpha,
    });
  }

  addPolygonFromDict(dict) {
    this.add_polygon(
        dict['polygon'],
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
