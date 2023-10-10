export class Frame {

  constructor() {
    this.drawList = [];
  }

  clear() {
    this.drawList = [];
  }

  add_circle(pos, radius, color, alpha = null) {
    this.drawList.push({
      type: "circle",
      pos: pos,
      radius: radius,
      color: color,
      alpha: alpha,
    });
  }

  add_circle_from_dict(dict) {
    add_circle(
        dict['pos'],
        dict['radius'],
        dict['color'],
        dict['alpha']
    )
  }

  add_polygon(polygon, color, alpha = null) {
    this.drawList.push({
      type: "polygon",
      polygons: polygon,
      color: color,
      alpha: alpha,
    });
  }

  add_line(line, lineWidth, color, alpha = null) {
    this.drawList.push({
      type: "line",
      lines: line,
      lineWidth: lineWidth,
      color: color,
      alpha: alpha,
    });
  }
}
