import xml.etree.ElementTree as ET
from scipy.spatial import ConvexHull
import numpy as np
from model.geometry.polygon import Polygon


NUM_CIRC_POINTS = 20


class URDFParser:

    @classmethod
    def _decompose_transformation_matrix(cls, matrix):

        # Extract the 3x3 rotation submatrix
        rotation_matrix = matrix[:3, :3]

        # Assuming XYZ rotation order (pitch, yaw, roll)
        # URDF and SDF use the X-Y-Z (Tait–Bryan) Extrinsic Euler Angle convention.
        # Calculate the rotation angles (Euler angles) in XYZ convention
        sx = np.sqrt(rotation_matrix[2, 1] * rotation_matrix[2, 1] + rotation_matrix[2, 2] * rotation_matrix[2, 2])
        singular = sx < 1e-6

        if not singular:
            x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            y = np.arctan2(-rotation_matrix[2, 0], sx)
            z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y = np.arctan2(-rotation_matrix[2, 0], sx)
            z = 0

            # Return angles in radians and translation vector
        return [x, y, z], matrix[:3, 3]

    @classmethod
    def parse_xyz(cls, xyz_str):
        xyz = list(map(float, xyz_str.split()))
        return np.array(xyz)

    @classmethod
    def create_transformation_matrix(cls, origin_xyz, axis_xyz):
        origin = URDFParser.parse_xyz(origin_xyz)
        axis = URDFParser.parse_xyz(axis_xyz)
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

    @classmethod
    def _parse_urdf(cls, path):

        tree = ET.parse(path)
        root = tree.getroot()

        links = {}
        for link in root.findall(".//link"):
            link_name = link.get("name")

            boxes = link.findall(".//visual/geometry/box")
            cylinders = link.findall(".//visual/geometry/cylinder")
            spheres = link.findall(".//visual/geometry/spgere")
            visuals = boxes + cylinders + spheres

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

    @classmethod
    def _apply_joint_transformations(cls, links_dict, joints_dict):

        # Add a global transformation matrix to each link
        for link_name in links_dict:
            links_dict[link_name]['transformation_matrix'] = np.identity(4)

        # Find the root: the root is never a child in the joint dict
        link_names = [key for key in links_dict.keys()]
        for joint_name, joint_info in joints_dict.items():
            joint_child = joints_dict[joint_name]['child_link']
            if joint_child in link_names:
                link_names.remove(joint_child)

        if len(link_names) != 1:
            raise ValueError('Malformed URDF file')

        root = link_names[0]

        # Add the used flag to each joint. Joints with flag == 'true'
        # will not be used. During the iteration of the algorithm
        # the joints will be put to used
        for joint, value in joints_dict.items():
            joints_dict[joint]['used'] = 'false'

        # Recursive call to apply the transformations to each link
        URDFParser._recursive_apply_joint_transformations(root, np.identity(4), links_dict, joints_dict)

        return links_dict

    @classmethod
    def _recursive_apply_joint_transformations(cls, link_name, transformation_matrix, links_dict, joints_dict):

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

                    child_transformation_matrix = URDFParser.create_transformation_matrix(origin_xyz, axis_xyz)
                    updated_transformation_matrix = np.dot(transformation_matrix, child_transformation_matrix)
                    links_dict[child_link_name]['transformation_matrix'] = updated_transformation_matrix

                    joint['used'] = 'true'

                    URDFParser._recursive_apply_joint_transformations(
                        child_link_name,
                        updated_transformation_matrix,
                        links_dict,
                        joints_dict
                    )

    @classmethod
    def _project_absolute_links(cls, links, discretization_points=NUM_CIRC_POINTS):

        polygons = []
        for link_name, link_properties in links.items():

            # Get link type
            link_geometry_type = link_properties['type']

            points = None
            if link_geometry_type == 'mesh':
                # Meshes are not supported
                raise ValueError('Mesh objects are not supported')

            elif link_geometry_type == 'box':

                dimensions = link_properties['dimensions'].split()
                width, height, depth = float(dimensions[0]), float(dimensions[1]), float(dimensions[2])

                points = np.array([
                    [-width / 2, -height / 2, -depth / 2],
                    [width / 2, -height / 2, -depth / 2],
                    [-width / 2, height / 2, -depth / 2],
                    [width / 2, height / 2, -depth / 2],
                    [-width / 2, -height / 2, depth / 2],
                    [width / 2, -height / 2, depth / 2],
                    [-width / 2, height / 2, depth / 2],
                    [width / 2, height / 2, depth / 2]
                ])

            elif link_geometry_type == 'cylinder':

                radius = float(link_properties['radius'])
                length = float(link_properties['length'])

                base = np.array(
                    [
                        [radius * np.cos(angle), radius * np.sin(angle), 0]
                        for angle in np.linspace(0, 2 * np.pi, discretization_points)
                    ]
                )
                points = np.concatenate((base, base))
                points[discretization_points // 2:, 2] = length

            elif link_geometry_type == 'sphere':

                radius = float(link_properties['radius'])

                points = np.array([
                        [radius * np.cos(angle), radius * np.sin(angle)]
                        for angle in np.linspace(0, 2 * np.pi, discretization_points)
                    ])

            # Decompose the transformation matrix
            transformation_matrix = link_properties['transformation_matrix']
            angles, translation_vector = URDFParser._decompose_transformation_matrix(transformation_matrix)

            # Get rotation matrix and translation vector
            rotation_matrix = transformation_matrix[:3, :3]
            translation_vector = transformation_matrix[:3, 3]

            # In case of the sphere, we can sum the translation and return
            if link_geometry_type == 'sphere':
                return points + translation_vector

            # In the other two cases, we need to go on with the algorithm

            # Rotate the points using the rotation matrix
            rotated_points = np.dot(points, rotation_matrix)

            # Project onto the XY plane
            projected_points = rotated_points[:, :2]

            # Take only the outermost among them
            hull = ConvexHull(projected_points)
            outermost_points = [projected_points[i] for i in hull.vertices]

            # Assign an angle to each of them (accounting for the center of the polygon)
            points_with_angles = []
            for point in outermost_points:
                angle = np.arctan2(point[1], point[0])
                points_with_angles.append((point[0], point[1], angle))

            # Sort the points based on the angles
            points = [[point[0], point[1]] for point in sorted(points_with_angles, key=lambda x: x[2])]

            # Add the translation
            for point in points:
                point[0] += translation_vector[0]
                point[1] += translation_vector[1]

            # Add the polygon to the array
            polygon = Polygon(points)
            polygons.append(polygon)

        return polygons

    @classmethod
    def parse(cls, pathlike):

        # Get links and joints dictionaries
        links, joints = URDFParser._parse_urdf(pathlike)

        # Recursively apply the joint transformations to each link
        links_absolute = URDFParser._apply_joint_transformations(links, joints)

        # Get the projection of each link on the XY plane
        polygons = URDFParser._project_absolute_links(links_absolute)

        return polygons


if __name__ == '__main__':

    urdf_path = "/home/daniel/Git/Robot-Simulator/model/world/URDF_parser/robot.urdf"
    polygons = URDFParser.parse(urdf_path)

    # Plot the projected XY points
    import matplotlib.pyplot as plt

    plt.figure()

    for polygon in polygons:
        x = [p.x for p in polygon.points]
        y = [p.y for p in polygon.points]
        plt.plot(x, y, color='blue')

    plt.title("Projected XY Points of Rotated 3D Box")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis('equal')  # Ensure aspect ratio is equal
    plt.show()
