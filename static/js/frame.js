export class Frame {

  constructor() {
    this.drawList = [];
  }

  clear() {
    this.drawList = [];
  }

  addCircle(pos, radius, lineWidth, fillColor, borderColor = null) {
    this.drawList.push({
      type: "circle",
      pos: pos,
      radius: radius,
      lineWidth: lineWidth,
      fillColor: fillColor,
      borderColor: borderColor,
    });
  }

  addCircleFromDict(dict) {
    this.addCircle(
        dict['pos'],
        dict['radius'],
        dict['line_width'],
        dict['fill_color'],
        dict['border_color']
    )
  }

  addPolygon(points, lineWidth, fillColor, borderColor = null) {
    this.drawList.push({
      type: "polygon",
      points: points,
      lineWidth: lineWidth,
      fillColor: fillColor,
      borderColor: borderColor,
    });
  }

  addPolygonFromDict(dict) {
    this.addPolygon(
        dict['points'],
        dict['line_width'],
        dict['fill_color'],
        dict['border_color']
    )
  }

  addLine(p1, p2, lineWidth, fillColor, borderColor = null) {
    this.drawList.push({
      type: "line",
      p1: p1,
      p2: p2,
      lineWidth: lineWidth,
      fillColor: fillColor,
      borderColor: borderColor,
    });
  }

  addLineFromDict(dict) {
    this.addLine(
        dict['p1'],
        dict['p2'],
        dict['line_width'],
        dict['fill_color'],
        dict['border_color']
    )
  }
}
