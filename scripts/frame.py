import json


class Frame:
    def __init__(self):
        self.draw_list = []

    def clear(self):
        self.draw_list = []

    def add_circle(self, pos, radius, color, alpha=None):
        self.draw_list.append(
            {
                "type": "circle",
                "pos": pos,
                "radius": radius,
                "color": color,
                "alpha": alpha,
            }
        )

    def add_circles(self, pos, radius, color, alpha=None):
        for p, r in zip(pos, radius):
            self.add_circle(p, r, color, alpha)

    def add_polygon(self, polygon, color, alpha=None):
        self.draw_list.append(
            {
                "type": "polygon",
                "polygons": polygon,
                "color": color,
                "alpha": alpha
            }
        )

    def add_polygons(self, polygons, color, alpha=None):
        for polygon in polygons:
            self.add_polygon(polygon, color, alpha)

    def add_line(self, line, line_width, color, alpha=None):
        self.draw_list.append(
            {
                "type": "line",
                "lines": line,
                "line_width": line_width,
                "color": color,
                "alpha": alpha,
            }
        )

    def add_lines(self, lines, line_widths, color, alpha=None):
        for line, line_width in zip(lines, line_widths):
            self.add_line(line, line_width, color, alpha)

    def to_json(self):
        return json.dumps(self.draw_list)
