"""
This module contains utility functions for visualizing 3D objects and geometric primitives using Open3D and Matplotlib.

Functions:
    - draw_plane_as_lines_open3d: Visualize a plane as a grid or lines in Open3D.
    - draw_face_of_cube: Draw a single face of a cube limited to specified bounds.
    - draw_cube: Draw a cube by combining six faces.
    - draw_line_extension_to_plane: Visualize a line and its intersection with a plane in 3D space.

Dependencies:
    - numpy
    - open3d
    - matplotlib
    - rsaitehu.geometry (custom module for geometric calculations)
"""
import numpy as np
import open3d as o3d
import rsaitehu_geometry as geom
import matplotlib.pyplot as plt

def draw_plane_as_lines_open3d(A, B, C, D, size=10, line_color=[1, 0, 0], grid_density=5, external_point=None):
    """
    Visualize a plane as a grid of lines using Open3D.

    The plane is represented by the equation Ax + By + Cz + D = 0, where (A, B, C) is
    the normal vector to the plane. The grid of lines visualizes the plane within a
    square area centered either on an external point projected onto the plane or
    at the origin by default.

    :param A: Coefficient A in the plane equation.
    :type A: float
    :param B: Coefficient B in the plane equation.
    :type B: float
    :param C: Coefficient C in the plane equation.
    :type C: float
    :param D: Coefficient D in the plane equation.
    :type D: float
    :param size: The extent of the grid in both x and y directions (default=10).
    :type size: float
    :param line_color: The RGB color of the grid lines (default=[1, 0, 0]).
    :type line_color: List[float]
    :param grid_density: The number of lines in each direction within the grid (default=5).
    :type grid_density: int
    :param external_point: A 3D point to project onto the plane and center the grid (default=None).
    :type external_point: Optional[np.ndarray]
    :return: An Open3D LineSet object representing the grid of lines visualizing the plane.
    :rtype: o3d.geometry.LineSet

    :raises ValueError: If C (the z-direction coefficient) is zero, making the plane undefined for visualization.

    :Example:

    ::

        >>> import open3d as o3d
        >>> plane_lines = draw_plane_as_lines_open3d(1, 0, -1, 0, size=5, grid_density=10)
        >>> o3d.visualization.draw_geometries([plane_lines])
    """

    vertices = []
    lines = []
    plane = np.array([A, B, C, D])

    # Centering grid on external point or origin
    center = (geom.get_point_of_plane_closest_to_given_point(plane, external_point)
              if external_point is not None else np.array([0, 0, 0]))
    center_x, center_y, center_z = center

    # Generate grid vertices
    for i in np.linspace(-size, size, grid_density):
        for j in np.linspace(-size, size, grid_density):
            x = center_x + i
            y = center_y + j
            z = -(D + A * x + B * y) / C
            vertices.append([x, y, z])

    # Create grid lines (horizontal and vertical)
    vertices = np.array(vertices)
    for i in range(grid_density):
        for j in range(grid_density - 1):
            # Horizontal lines
            lines.append([i * grid_density + j, i * grid_density + j + 1])
            # Vertical lines
            lines.append([j * grid_density + i, (j + 1) * grid_density + i])

    # Create LineSet object
    lineset = o3d.geometry.LineSet()
    lineset.points = o3d.utility.Vector3dVector(vertices)
    lineset.lines = o3d.utility.Vector2iVector(lines)
    lineset.colors = o3d.utility.Vector3dVector([line_color] * len(lines))

    return lineset

def draw_face_of_cube(plane: np.ndarray, cube_min: np.ndarray, cube_max: np.ndarray, color: str = "red", alpha: float = 0.5, ax=None):
    """
    Visualize a plane face within the bounds of a cube using Matplotlib.

    The function plots a plane limited to the cube's bounds. If no axis is provided, 
    a new Matplotlib figure is created. The plane is defined by its equation 
    Ax + By + Cz + D = 0.

    :param plane: A numpy array [A, B, C, D] representing the coefficients of the plane equation.
    :type plane: np.ndarray
    :param cube_min: A numpy array [x_min, y_min, z_min] representing the cube's minimum bounds.
    :type cube_min: np.ndarray
    :param cube_max: A numpy array [x_max, y_max, z_max] representing the cube's maximum bounds.
    :type cube_max: np.ndarray
    :param color: The color of the plane face (default="red").
    :type color: str
    :param alpha: The transparency level of the plane face (default=0.5).
    :type alpha: float
    :param ax: A Matplotlib 3D axis to plot on. If None, a new figure and axis are created (default=None).
    :type ax: Axes3D or None
    :return: None

    :raises ValueError: If the plane coefficients are invalid or cannot be resolved for plotting.

    :Example:

    ::

        >>> import numpy as np
        >>> import matplotlib.pyplot as plt
        >>> from module import draw_face_of_cube
        >>> plane = np.array([0, 0, 1, -3])  # Plane: z = 3
        >>> cube_min = np.array([-2, -2, -2])
        >>> cube_max = np.array([2, 2, 2])
        >>> fig = plt.figure()
        >>> ax = fig.add_subplot(111, projection='3d')
        >>> draw_face_of_cube(plane, cube_min, cube_max, color="blue", alpha=0.5, ax=ax)
        >>> plt.show()
    """
    create_plot = ax is None
    if create_plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    A, B, C, D = plane
    tol = 1e-6  # tolerance to avoid floating-point issues

    # Choose independent variables based on which coefficient is safely nonzero.
    if abs(C) > tol:
        # Use x and y as independent variables, solve for z.
        x_vals = np.linspace(cube_min[0], cube_max[0], 50)
        y_vals = np.linspace(cube_min[1], cube_max[1], 50)
        x_grid, y_grid = np.meshgrid(x_vals, y_vals)
        z_grid = (-A * x_grid - B * y_grid - D) / C
    elif abs(A) > tol:
        # Use y and z as independent variables, solve for x.
        y_vals = np.linspace(cube_min[1], cube_max[1], 50)
        z_vals = np.linspace(cube_min[2], cube_max[2], 50)
        y_grid, z_grid = np.meshgrid(y_vals, z_vals)
        x_grid = (-B * y_grid - C * z_grid - D) / A
    elif abs(B) > tol:
        # Use x and z as independent variables, solve for y.
        x_vals = np.linspace(cube_min[0], cube_max[0], 50)
        z_vals = np.linspace(cube_min[2], cube_max[2], 50)
        x_grid, z_grid = np.meshgrid(x_vals, z_vals)
        y_grid = (-A * x_grid - C * z_grid - D) / B
    else:
        raise ValueError("Invalid plane: all coefficients are nearly zero.")

    ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=alpha)

    if create_plot:
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

def draw_cube(cube_min: np.ndarray, cube_max: np.ndarray, color: str = "red", alpha: float = 0.5, ax=None):
    """
    Visualize a cube in 3D space using Matplotlib by combining its six faces.

    The cube is defined by its minimum and maximum corner points. Each face is drawn
    using the `draw_face_of_cube` function. If no axis is provided, a new Matplotlib
    figure is created.

    :param cube_min: A numpy array [x_min, y_min, z_min] representing the cube's minimum bounds.
    :type cube_min: np.ndarray
    :param cube_max: A numpy array [x_max, y_max, z_max] representing the cube's maximum bounds.
    :type cube_max: np.ndarray
    :param color: The color of the cube's faces (default="red").
    :type color: str
    :param alpha: The transparency level of the cube's faces (default=0.5).
    :type alpha: float
    :param ax: A Matplotlib 3D axis to plot on. If None, a new figure and axis are created (default=None).
    :type ax: Axes3D or None
    :return: None

    :raises ValueError: If cube_min or cube_max are not valid 3D coordinates.

    :Example:

    ::

        >>> import numpy as np
        >>> import matplotlib.pyplot as plt
        >>> from module import draw_cube
        >>> cube_min = np.array([-1, -1, -1])
        >>> cube_max = np.array([1, 1, 1])
        >>> fig = plt.figure()
        >>> ax = fig.add_subplot(111, projection='3d')
        >>> draw_cube(cube_min, cube_max, color="green", alpha=0.7, ax=ax)
        >>> plt.show()
    """
    create_plot = ax is None
    if create_plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    planes = [
        [1, 0, 0, -cube_min[0]],   # Front face
        [-1, 0, 0, cube_max[0]],   # Back face
        [0, 1, 0, -cube_min[1]],   # Top face
        [0, -1, 0, cube_max[1]],   # Bottom face
        [0, 0, 1, -cube_min[2]],   # Left face
        [0, 0, -1, cube_max[2]]    # Right face
    ]

    for plane in planes:
        draw_face_of_cube(plane, cube_min, cube_max, color=color, alpha=alpha, ax=ax)

    if create_plot:
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

def draw_line_extension_to_plane(line: np.ndarray, plane: np.ndarray, ax=None):
    """
    Visualize a line and its intersection with a plane in 3D space using Matplotlib.

    The function plots a line segment in 3D and highlights its intersection with a plane
    defined by the equation Ax + By + Cz + D = 0. If no axis is provided, a new Matplotlib
    figure is created.

    :param line: A numpy array of shape (2, 3) representing two endpoints of the line.
    :type line: np.ndarray
    :param plane: A numpy array [A, B, C, D] representing the coefficients of the plane equation.
    :type plane: np.ndarray
    :param ax: A Matplotlib 3D axis to plot on. If None, a new figure and axis are created (default=None).
    :type ax: Axes3D or None
    :return: None

    :raises ValueError: If the line or plane input dimensions are incorrect or if the intersection cannot be computed.

    :Example:

    ::

        >>> import numpy as np
        >>> import matplotlib.pyplot as plt
        >>> from module import draw_line_extension_to_plane
        >>> line = np.array([[0, 0, 0], [1, 1, 1]])  # Line segment
        >>> plane = np.array([0, 0, 1, -3])  # Plane: z = 3
        >>> fig = plt.figure()
        >>> ax = fig.add_subplot(111, projection='3d')
        >>> draw_line_extension_to_plane(line, plane, ax=ax)
        >>> plt.show()
    """
    create_plot = ax is None
    if create_plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    # Compute the intersection point of the line and plane
    intersection = geom.get_intersection_point_of_line_with_plane(line, plane)

    if intersection is None:
        raise ValueError("The line does not intersect the plane.")

    # Plot the line segment
    ax.plot(*zip(*line), linestyle='-', color='blue', label='Line')

    # Highlight the intersection point
    ax.scatter(*intersection, color='green', label='Intersection', s=100)

    if create_plot:
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()



