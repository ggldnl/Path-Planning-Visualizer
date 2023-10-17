import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from model.geometry.point import Point
from model.geometry.polygon import Polygon
from model.world.URDF_parser.URDF_shape_projection import Box, Cylinder, Sphere, decompose_transformation_matrix
import numpy as np


NUM_CIRC_POINTS = 8


def apply_joint_transformations(link_dict, joint_dict, root=None):

    # Add a global transformation matrix to each link
    for link_name in link_dict:
        link_dict[link_name]['transformation_matrix'] = np.identity(4)

    # Find the root: the root is never a child in the joint dict
    if root is None:
        link_names = [key for key in link_dict.keys()]
        for joint_name, joint_info in joint_dict.items():
            joint_child = joint_dict[joint_name]['child_link']
            if joint_child in link_names:
                link_names.remove(joint_child)

        if len(link_names) != 1:
            raise ValueError('Malformed URDF file')

        root = link_names[0]

    # Add the used flag to each joint. Joints with flag == 'true'
    # will not be used. During the iteration of the algorithm
    # the joints will be put to used
    for joint, value in joint_dict.items():
        joint_dict[joint]['used'] = 'false'

    _apply_joint_transformations(root, np.identity(4), link_dict, joint_dict)


def parse_xyz(xyz_str):
    xyz = list(map(float, xyz_str.split()))
    return np.array(xyz)


def create_transformation_matrix(origin_xyz, axis_xyz):
    origin = parse_xyz(origin_xyz)
    axis = parse_xyz(axis_xyz)
    x, y, z = axis
    c = np.cos(np.arctan2(y, x))
    s = np.sin(np.arctan2(y, x))
    t = np.arctan2(np.sqrt(x ** 2 + y ** 2), z)

    transformation_matrix = np.array([
        [c, -s * c, s * s * t, origin[0]],
        [s, c * c, -s * c * t, origin[1]],
        [0, s * t, c, origin[2]],
        [0, 0, 0, 1]
    ])

    return transformation_matrix


def _apply_joint_transformations(link_name, transformation_matrix, links_dict, joints_dict):

    if link_name in links_dict:
        links_dict[link_name]['transformation_matrix'] = transformation_matrix
        for joint_name in joints_dict.keys():

            joint = joints_dict[joint_name]

            # Take a joint that has the selected link as parent
            if joint['parent_link'] == link_name and joint['used'] == 'false':
                # Update its transformation matrix

                child_link_name = joint['child_link']
                origin_xyz = joint['origin']['xyz']
                axis_xyz = joint['axis']['xyz']

                # child_transformation_matrix = links_dict[child_link_name]['transformation_matrix']
                child_transformation_matrix = create_transformation_matrix(origin_xyz, axis_xyz)
                updated_transformation_matrix = np.dot(transformation_matrix, child_transformation_matrix)
                links_dict[child_link_name]['transformation_matrix'] = updated_transformation_matrix

                joint['used'] = 'true'

                _apply_joint_transformations(child_link_name, updated_transformation_matrix, links_dict, joints_dict)


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

    joints = {}
    for joint in root.findall(".//joint"):

        joint_name = joint.get("name")
        parent_link = joint.find(".//parent").get("link")
        child_link = joint.find(".//child").get("link")

        # Store joint information in joint_dict
        joints[joint_name] = {
            "type": joint.get("type"),
            "parent_link": parent_link,
            "child_link": child_link,
            "origin": joint.find(".//origin").attrib,  # Store rotation
            "axis": joint.find(".//axis").attrib,  # Store translation
        }

    return links, joints


def parse_links(links, joints):

    polygons = []
    for link_name, link_properties in links.items():

        # Get link type
        link_geometry_type = link_properties['type']

        if link_geometry_type == 'mesh':
            # Meshes are not supported
            raise ValueError('Mesh objects are not supported')

        elif link_geometry_type == 'box':

            # Build the box
            dimensions = link_properties['dimensions'].split()
            width, length, depth = float(dimensions[0]), float(dimensions[1]), float(dimensions[2])
            box = Box(width, length, depth)

            # Get the transform
            box_transform = link_properties['transformation_matrix']
            angles, translation_vector = decompose_transformation_matrix(box_transform)
            box_polygon = box.get_projection(angles, translation_vector)

            polygons.append(box_polygon)

        elif link_geometry_type == 'sphere':

            # Build the sphere
            radius = float(link_properties['radius'])
            sphere = Sphere(radius)

            # Get the transform
            sphere_transform = link_properties['transformation_matrix']
            angles, translation_vector = decompose_transformation_matrix(sphere_transform)
            sphere_polygon = sphere.get_projection(angles, translation_vector)

            polygons.append(sphere_polygon)

        elif link_geometry_type == 'cylinder':

            # Build the cylinder
            radius = float(link_properties['radius'])
            length = float(link_properties['length'])
            cylinder = Cylinder(radius, length)

            # Get the transform
            cylinder_transform = link_properties['transformation_matrix']
            angles, translation_vector = decompose_transformation_matrix(cylinder_transform)
            cylinder_polygon = cylinder.get_projection(angles, translation_vector)

            polygons.append(cylinder_polygon)

    return polygons


def main():

    urdf_file = "/home/daniel/Git/Robot-Simulator/model/world/URDF_parser/robot.urdf"

    # Get links and joints dictionaries
    links, joints = parse_urdf(urdf_file)

    # Recursively apply the joint transformations to each link
    apply_joint_transformations(links, joints)

    # Print the updated links dictionary
    for link_name in links:
        print(f"Link: {link_name}")
        print("Transformation Matrix:")
        print(links[link_name]['transformation_matrix'])
        print("\n")

    # Get the projection of each link on the XY plane
    polygons = parse_links(links, joints)

    # Plot the top view polygon
    plt.figure()

    colors = ['blue', 'red']
    for polygon, color in zip(polygons, colors):
        x = [point[0] for point in polygon]
        y = [point[1] for point in polygon]
        plt.plot(x, y, color=color)

    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title("Top View of Robot Polygon")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid()
    plt.show()


if __name__ == "__main__":
    main()
