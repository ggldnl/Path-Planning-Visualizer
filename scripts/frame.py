import json


class Frame:
    def __init__(self):
        self.draw_list = []

    def clear(self):
        self.draw_list = []

    def add_circle(self, pos, radius, fill_color, border_color=None):
        self.draw_list.append(
            {
                "type": "circle",
                "pos": pos,
                "radius": radius,
                "fill_color": fill_color,
                "border_color": fill_color if border_color is None else border_color
            }
        )

    def add_circles(self, pos, radius, fill_color, border_color=None):
        for p, r in zip(pos, radius):
            self.add_circle(p, r, fill_color, border_color)

    def add_polygon(self, polygon, fill_color, border_color=None):
        self.draw_list.append(
            {
                "type": "polygon",
                "points": polygon.to_point_array(),
                "fill_color": fill_color,
                "border_color": fill_color if border_color is None else border_color
            }
        )

    def add_polygons(self, polygons, fill_color, border_color=None):
        for polygon in polygons:
            self.add_polygon(polygon, fill_color, border_color)

    def add_line(self, line, line_width, fill_color, border_color=None):
        self.draw_list.append(
            {
                "type": "line",
                "points": line,  # TODO uniform with polygon
                "line_width": line_width,
                "fill_color": fill_color,
                "border_color": border_color,
            }
        )

    def add_lines(self, lines, line_widths, fill_color, border_color=None):
        for line, line_width in zip(lines, line_widths):
            self.add_line(line, line_width, fill_color, border_color)

    def to_json(self):
        return json.dumps(self.draw_list)


if __name__ == '__main__':

    frame = Frame()
    frame.add_circle((0, 0), 10, 'red')
    frame.add_circle((30, 30), 10, 'orange')
    json_string = frame.to_json()
    print(json_string)
