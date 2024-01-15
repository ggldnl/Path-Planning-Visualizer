import xml.etree.ElementTree as ET
from scipy.spatial import ConvexHull
import numpy as np
from model.geometry.polygon import Polygon


NUM_CIRC_POINTS = 20


class URDFParser:

    @classmethod
    def _pretty_dot_print(cls, left_matrix, right_matrix, left_name=None, right_name=None):

        assert left_matrix.shape == right_matrix.shape

        result = left_matrix.dot(right_matrix)

        print(f'{left_name}{" "* 44}{right_name}')

        for i in range(len(left_matrix)):
            print('|', end='')
            for j in range(len(left_matrix[i])):
                print(f'{left_matrix[i][j]:2.6f}   ', end='')
            print('|', end='')
            if i == len(left_matrix) // 2:
                print(f'   @   ', end='')
            else:
                print('       ', end='')
            print('|', end='')
            for j in range(len(right_matrix[i])):
                print(f'{right_matrix[i][j]:2.6f}   ', end='')
            print('|', end='')
            if i == len(left_matrix) // 2:
                print(f'   =   ', end='')
            else:
                print('       ', end='')
            print('|', end='')
            for j in range(len(result[i])):
                print(f'{result[i][j]:2.6f}   ', end='')
            print('|', end='')
            print()
        print()

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
        seq = list(map(float, xyz_str.split()))
        return seq[0], seq[1], seq[2]

    @classmethod
    def create_transformation_matrix(cls, origin_xyz, origin_rpy):

        x, y, z = origin_xyz
        roll, pitch, yaw = origin_rpy

        # Create the translation matrix
        translation_matrix = np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

        # Create the rotation matrix using RPY angles
        rotation_matrix = np.array([
            [np.cos(yaw) * np.cos(pitch), np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll),
             np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll), 0],
            [np.sin(yaw) * np.cos(pitch), np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll),
             np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll), 0],
            [-np.sin(pitch), np.cos(pitch) * np.sin(roll), np.cos(pitch) * np.cos(roll), 0],
            [0, 0, 0, 1]
        ])

        # Combine translation and rotation to get the transformation matrix.
        # The order in which we do the product matters. For translation first
        # and then rotation we would multiply the translation matrix by the
        # rotation matrix (T * R). If we want to first apply the rotation
        # and then the translation, we would want to multiply the rotation
        # matrix by the translation matrix (R * T)
        transformation_matrix = np.dot(translation_matrix, rotation_matrix)

        return transformation_matrix

    @classmethod
    def create_rotation_matrix(cls, origin_rpy):

        roll, pitch, yaw = origin_rpy

        # Compute the elements of the combined rotation matrix
        cos_phi, sin_phi = np.cos(roll), np.sin(roll)
        cos_theta, sin_theta = np.cos(pitch), np.sin(pitch)
        cos_psi, sin_psi = np.cos(yaw), np.sin(yaw)

        # Compute the combined rotation matrix
        R = np.array([
            [cos_theta * cos_psi, -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi,
             sin_phi * sin_psi + cos_phi * sin_theta * cos_psi],
            [cos_theta * sin_psi, cos_phi * cos_psi + sin_phi * sin_theta * sin_psi,
             -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi],
            [-sin_theta, sin_phi * cos_theta, cos_phi * cos_theta]
        ])

        return R

    @classmethod
    def create_translation_vector(cls, origin_xyz):

        tx, ty, tz = origin_xyz

        T = np.array([
            tx, ty, tz
        ])

        return T

    @classmethod
    def _parse_urdf(cls, root):

        # root = tree.getroot()

        links = {}
        for link in root.findall(".//link"):
            link_name = link.get("name")

            # Find the visual element
            boxes = link.findall(".//visual/geometry/box")
            cylinders = link.findall(".//visual/geometry/cylinder")
            spheres = link.findall(".//visual/geometry/sphere")
            visuals = boxes + cylinders + spheres

            # Should have only one visual element
            if len(visuals) > 1:
                raise ValueError('Malformed URDF file')
            visual = visuals[0]

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

            # Do not consider mesh objects

            # Add the transformation matrix for the link
            links[link_name]["transformation_matrix"] = np.identity(4)

            # Some links can have a translation or rotation
            origins = link.findall(".//visual/origin")
            if len(origins) > 1:
                raise ValueError('Malformed URDF file')

            if len(origins) > 0:
                origin = origins[0]

                # We must account for a different initial transformation
                link_xyz = origin.attrib.get('xyz')
                link_rpy = origin.attrib.get('rpy')
                links[link_name]["transformation_matrix"] = URDFParser.create_transformation_matrix(
                    URDFParser.parse_xyz(link_xyz),
                    URDFParser.parse_xyz(link_rpy)
                )

        joints = {}
        for joint in root.findall(".//joint"):

            joint_name = joint.get("name")
            joint_type = joint.get("type")
            parent_link = joint.find(".//parent").get("link")
            child_link = joint.find(".//child").get("link")

            if parent_link in links and child_link in links:

                # The origin element is used to define the position and orientation of a
                # particular element within the robot model. It specifies a reference frame
                # for that element and allows you to position and orient the element
                # relative to that reference frame.
                # Its attributes are xyz and rpy
                joint_origin = joint.find(".//origin").attrib
                joint_origin_xyz = joint_origin.get('xyz')
                joint_origin_rpy = joint_origin.get('rpy')

                # Maybe there is no rotation
                if joint_origin_rpy is None:
                    joint_origin_rpy = "0.0 0.0 0.0"

                # The axis element is used to define the orientation of an axis of rotation
                # for a joint. It is typically used within joint descriptions to specify
                # the axis of rotation for that joint. The axis element has attributes for
                # specifying the orientation of the axis, that is xyz.
                # If the axis were {'xyz': '0 0 1'} it would have meant that the joint has
                # a rotation axis along the Z-axis of its reference frame.
                # joint_axis = joint.find(".//axis")

                # Store joint information in joint_dict
                joints[joint_name] = {
                    "type": joint_type,
                    "parent_link": parent_link,
                    "child_link": child_link,
                    "origin_xyz": joint_origin_xyz,
                    "origin_rpy": joint_origin_rpy
                }

        return links, joints

    @classmethod
    def _apply_joint_transformations(cls, links_dict, joints_dict):

        # Find the root: the root is never a child in the joint dict
        link_names = [key for key in links_dict.keys()]
        for joint_name, joint_info in joints_dict.items():
            joint_child = joints_dict[joint_name]['child_link']
            if joint_child in link_names:
                link_names.remove(joint_child)

        if len(link_names) != 1:
            raise ValueError('Malformed URDF file: two or more links are not joined to anything')

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
    def _recursive_apply_joint_transformations(cls, link_name, parent_transformation_matrix, links_dict, joints_dict):

        if link_name in links_dict:

            for joint_name in joints_dict.keys():

                joint = joints_dict[joint_name]

                # Take a joint that has the selected link as parent
                if joint['parent_link'] == link_name and joint['used'] == 'false':

                    # Update its transformation matrix
                    child_link_name = joint['child_link']
                    origin_xyz = joint['origin_xyz']
                    origin_rpy = joint['origin_rpy']

                    link_transformation_matrix = URDFParser.create_transformation_matrix(
                        URDFParser.parse_xyz(origin_xyz),
                        URDFParser.parse_xyz(origin_rpy)
                    )

                    # This is the transformation matrix that will be streamlined to the next joint
                    updated_transformation_matrix = np.dot(parent_transformation_matrix, link_transformation_matrix)

                    # The link has another transformation matrix that does not need to be streamlined to the next link
                    child_transformation_matrix = links_dict[child_link_name]['transformation_matrix']
                    absolute_child_transformation = np.dot(updated_transformation_matrix, child_transformation_matrix)
                    links_dict[child_link_name]['transformation_matrix'] = absolute_child_transformation

                    # Consume the joint
                    joint['used'] = 'true'

                    # Next
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
                        [radius * np.cos(angle), radius * np.sin(angle), -length / 2]
                        for angle in np.linspace(0, 2 * np.pi, discretization_points)
                    ]
                )
                points = np.concatenate((base, base))
                points[discretization_points:, 2] = length / 2

            elif link_geometry_type == 'sphere':

                radius = float(link_properties['radius'])

                points = np.array([
                        [radius * np.cos(angle), radius * np.sin(angle)]
                        for angle in np.linspace(0, 2 * np.pi, discretization_points)
                    ])

            # Decompose the transformation matrix
            transformation_matrix = link_properties['transformation_matrix']

            # Get rotation matrix and translation vector
            rotation_matrix = transformation_matrix[:3, :3]
            translation_vector = transformation_matrix[:3, 3]

            # In case of the sphere, we can sum the translation and return
            if link_geometry_type == 'sphere':
                polygons.append(Polygon((points + translation_vector[:2]).tolist()))
                continue

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

    @staticmethod
    def _compute_convex_hull(polygon_list):
        points = []
        for body in polygon_list:
            for point in body.points:
                points.append(point.to_array())

        # Take only the outermost among them
        hull = ConvexHull(points)
        outermost_points = [points[i] for i in hull.vertices]
        return Polygon(outermost_points)

    @classmethod
    def _parse_data(cls, root):

        # Get links and joints dictionaries
        links, joints = URDFParser._parse_urdf(root)

        # Recursively apply the joint transformations to each link
        links_absolute = URDFParser._apply_joint_transformations(links, joints)

        # Get the projection of each link on the XY plane
        polygons = URDFParser._project_absolute_links(links_absolute)

        # Compute the convex hull of the polygons
        outline = URDFParser._compute_convex_hull(polygons)

        # Compute the boundaries of the convex hull
        bounds = outline.get_bounds()

        # Use the width as wheelbase (minx, miny, maxx, maxy)
        width = bounds[2] - bounds[0]

        return polygons, width

    @classmethod
    def parse_string(cls, xml_string):

        root = ET.fromstring(xml_string)

        return URDFParser._parse_data(root)

    @classmethod
    def parse_path(cls, pathlike):

        tree = ET.parse(pathlike)
        root = tree.getroot()

        return URDFParser._parse_data(root)


if __name__ == '__main__':

    # Test urdf file in local folder
    urdf_path = "robots/R2D2/R2D2.urdf"
    polygons = URDFParser.parse(urdf_path)

    # Plot the projected XY points
    import matplotlib.pyplot as plt

    plt.figure()

    for polygon in polygons:
        x = [p.x for p in polygon.points]
        y = [p.y for p in polygon.points]
        plt.plot(x, y, color='blue')

    plt.title("Projected XY Points")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis('equal')  # Ensure aspect ratio is equal
    plt.show()
