import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
# from shapely.geometry import Polygon, Point
from model.geometry.point import Point
from model.geometry.polygon import Polygon
import numpy as np


NUM_CIRC_POINTS = 8


def parse_urdf(urdf_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    links = {}
    for link in root.findall(".//link"):
        link_name = link.get("name")

        visuals = link.findall(".//visual/geometry/box") + \
                  link.findall(".//visual/geometry/cylinder") + \
                  link.findall(".//visual/geometry/spgere")

        for visual in visuals:
            geometry_type = visual.tag
            if geometry_type == "box":
                dimensions = visual.attrib['size']
                links[link_name] = {"type": "box", "dimensions": dimensions}
            elif geometry_type == "cylinder":
                radius = visual.attrib['radius']
                length = visual.attrib['length']
                links[link_name] = {"type": "cylinder", "radius": radius, "length": length}
            elif geometry_type == "sphere":
                radius = visual.attrib['radius']
                links[link_name] = {"type": "sphere", "radius": radius}
            break  # Only consider the first visual element

    return links


def parse_links(links):

    """
    example:
    links = {
        'link1', {'type': 'box', 'dimensions': '0.2 0.2 0.2'},
        'link2', {'type': 'cylinder', 'radius': '0.1', 'length': '0.4'}
    }
    """

    polygons = []
    for link_name, link_properties in links.items():

        # Get link type
        link_geometry_type = link_properties['type']

        if link_geometry_type == 'mesh':
            # Meshes are not supported
            pass
        elif link_geometry_type == 'box':
            dimensions = link_properties['dimensions'].split()
            width, height, depth = float(dimensions[0]), float(dimensions[1]), float(dimensions[2])
            polygons.append(Polygon(
                [
                    Point(0, 0),
                    Point(0, height),
                    Point(width, height),
                    Point(width, 0),
                    Point(0, 0)
                ]
            ))
        elif link_geometry_type == 'sphere':
            radius = float(link_properties['radius'])
            polygons.append(Polygon(
                [
                    Point(radius * np.cos(angle), radius * np.sin(angle))
                    for angle in np.linspace(0, 2 * np.pi, NUM_CIRC_POINTS)
                ] + [Point(radius, 0)]
            ))
        elif link_geometry_type == 'cylinder':
            radius = float(link_properties['radius'])
            polygons.append(Polygon(
                [
                    Point(radius * np.cos(angle), radius * np.sin(angle))
                    for angle in np.linspace(0, 2 * np.pi, NUM_CIRC_POINTS)
                ] + [Point(radius, 0)]
            ))

    return polygons


def main():
    urdf_file = "/home/daniel/Git/Robot-Simulator/model/world/robot/robot.urdf"

    links = parse_urdf(urdf_file)
    polygons = parse_links(links)

    # Plot the top view polygon
    plt.figure()

    print(polygons)

    colors = ['blue', 'red']
    for polygon, color in zip(polygons, colors):
        x = [point.x for point in polygon.points]
        y = [point.y for point in polygon.points]
        plt.plot(x, y, color=color)

    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Top View of Robot Polygon")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid()
    plt.show()


if __name__ == "__main__":
    main()
