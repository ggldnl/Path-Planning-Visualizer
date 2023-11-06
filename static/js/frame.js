export class Frame {

  constructor() {
    this.drawList = [];
  }

  clear() {
    this.drawList = [];
  }

  addCircle(pos, radius, fillColor, borderColor = null) {
    this.drawList.push({
      type: "circle",
      pos: pos,
      radius: radius,
      fillColor: fillColor,
      borderColor: borderColor,
    });
  }

  addCircleFromDict(dict) {
    this.addCircle(
        dict['pos'],
        dict['radius'],
        dict['fill_color'],
        dict['border_color']
    )
  }

  addPolygon(points, fillColor, borderColor = null) {
    this.drawList.push({
      type: "polygon",
      points: points,
      fillColor: fillColor,
      borderColor: borderColor,
    });
  }

  addPolygonFromDict(dict) {
    this.addPolygon(
        dict['points'],
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
