import math
from vector import Vector  # Supondo que a classe Vector seja definida em outro arquivo

def degrees(theta_radians):
    return (theta_radians * 180) / math.pi

def radians(theta_degrees):
    return (theta_degrees * math.pi) / 180

def is_triangle(a, b, c):
    return a + b > c and a + c > b and b + c > a

def dot(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z

def vector_length(v):
    return math.sqrt(dot(v, v))

def is_counter_clockwise(a, b, n):
    return dot(a, cross(b, n)) > 0

def vector_from_to(a, b):
    return Vector(b.x - a.x, b.y - a.y, b.z - a.z)

def scale_vector(v, d):
    return Vector(d * v.x, d * v.y, d * v.z)

def add_vectors(a, b):
    return Vector(a.x + b.x, a.y + b.y, a.z + b.z)

def get_unit_vector(v):
    return scale_vector(v, 1 / vector_length(v))

def cross(a, b):
    x = a.y * b.z - a.z * b.y
    y = a.z * b.x - a.x * b.z
    z = a.x * b.y - a.y * b.x
    return Vector(x, y, z)

def get_normal_of_three_points(a, b, c):
    ab = vector_from_to(a, b)
    ac = vector_from_to(a, c)
    n = cross(ab, ac)
    len_n = vector_length(n)
    unit_n = scale_vector(n, 1 / len_n)

    return unit_n

def acos_degrees(ratio):
    theta_radians = math.acos(ratio)

    # mimics behavior of Python numpy acos
    if math.isnan(theta_radians):
        return 0

    return degrees(theta_radians)

def angle_opposite_of_last_side(a, b, c):
    if a == 0 or b == 0:
        return None

    cos_theta = (a * a + b * b - c * c) / (2 * a * b)
    return acos_degrees(cos_theta)

def angle_between(a, b):
    if vector_length(a) == 0 or vector_length(b) == 0:
        return 0

    cos_theta = dot(a, b) / math.sqrt(dot(a, a) * dot(b, b))
    return acos_degrees(cos_theta)

# u is the vector, n is the plane normal
def projected_vector_onto_plane(u, n):
    s = dot(u, n) / dot(n, n)
    temp_vector = scale_vector(n, s)
    return vector_from_to(temp_vector, u)

def get_sin_cos(theta):
    return [math.sin(radians(theta)), math.cos(radians(theta))]

IDENTITY_MATRIX_4x4 = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
]

def uniform_matrix_4x4(d):
    d_row = [d, d, d, d]
    return [d_row[:], d_row[:], d_row[:], d_row[:]]

def add(a, b):
    return a + b

def multiply(a, b):
    return a * b

def operate_4x4(matrix_a, matrix_b, operation):
    result_matrix = uniform_matrix_4x4(None)
    for i in range(4):
        for j in range(4):
            result_matrix[i][j] = operation(matrix_a[i][j], matrix_b[i][j])
    return result_matrix

def dot_multiply_4x4(matrix_a, matrix_b):
    return operate_4x4(matrix_a, matrix_b, multiply)

def add_4x4(matrix_a, matrix_b):
    return operate_4x4(matrix_a, matrix_b, add)

def multiply_4x4(matrix_a, matrix_b):
    result_matrix = uniform_matrix_4x4(None)

    for i in range(4):
        for j in range(4):
            result_matrix[i][j] = (
                matrix_a[i][0] * matrix_b[0][j] +
                matrix_a[i][1] * matrix_b[1][j] +
                matrix_a[i][2] * matrix_b[2][j] +
                matrix_a[i][3] * matrix_b[3][j]
            )

    return result_matrix

def t_rot_x_matrix(theta, tx=0, ty=0, tz=0):
    s, c = get_sin_cos(theta)
    return [
        [1, 0, 0, tx],
        [0, c, -s, ty],
        [0, s, c, tz],
        [0, 0, 0, 1],
    ]

def t_rot_y_matrix(theta, tx=0, ty=0, tz=0):
    s, c = get_sin_cos(theta)
    return [
        [c, 0, s, tx],
        [0, 1, 0, ty],
        [-s, 0, c, tz],
        [0, 0, 0, 1],
    ]

def t_rot_z_matrix(theta, tx=0, ty=0, tz=0):
    s, c = get_sin_cos(theta)
    return [
        [c, -s, 0, tx],
        [s, c, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1],
    ]

def t_rot_xyz_matrix(x_theta, y_theta, z_theta):
    rx = t_rot_x_matrix(x_theta)
    ry = t_rot_y_matrix(y_theta)
    rz = t_rot_z_matrix(z_theta)
    rxy = multiply_4x4(rx, ry)
    rxyz = multiply_4x4(rxy, rz)
    return rxyz

def skew(p):
    return [
        [0, -p.z, p.y, 0],
        [p.z, 0, -p.x, 0],
        [-p.y, p.x, 0, 0],
        [0, 0, 0, 1],
    ]

def matrix_to_align_vector_a_to_b(a, b):
    v = cross(a, b)
    s = vector_length(v)
    # When the angle between a and b is zero or 180 degrees
    # then cross product is 0, R = I
    if s == 0:
        return IDENTITY_MATRIX_4x4

    c = dot(a, b)
    vx = skew(v)
    d = (1 - c) / (s * s)
    vx2 = multiply_4x4(vx, vx)
    d_matrix = uniform_matrix_4x4(d)
    dvx2 = dot_multiply_4x4(vx2, d_matrix)
    temp = add_4x4(IDENTITY_MATRIX_4x4, vx)
    transform_matrix = add_4x4(temp, dvx2)
    return transform_matrix

# Expondo as funções
__all__ = [
    "degrees", "radians", "is_triangle", "dot", "cross", "get_normal_of_three_points", 
    "scale_vector", "vector_from_to", "add_vectors", "get_unit_vector", 
    "projected_vector_onto_plane", "vector_length", "angle_between", 
    "angle_opposite_of_last_side", "is_counter_clockwise", "t_rot_x_matrix", 
    "t_rot_y_matrix", "t_rot_z_matrix", "t_rot_xyz_matrix", "skew", 
    "matrix_to_align_vector_a_to_b", "multiply_4x4"
]
