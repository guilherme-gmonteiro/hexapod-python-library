class Vector:
    def __init__(self, x, y, z, name="no-name-point", id="no-id-point"):
        self.x = x
        self.y = y
        self.z = z
        self.name = name
        self.id = id

    def new_trot(self, transform_matrix, name="unnamed-point", id="no-id"):
        """
        Given point location with respect to local axes coordinate frame,
        find the point in a global axes coordinate frame,
        where the local axes with respect to the global frame are defined
        by the parameter transform_matrix.
        """
        r0, r1, r2 = transform_matrix[:3]
        r00, r01, r02, tx = r0
        r10, r11, r12, ty = r1
        r20, r21, r22, tz = r2

        new_x = self.x * r00 + self.y * r01 + self.z * r02 + tx
        new_y = self.x * r10 + self.y * r11 + self.z * r12 + ty
        new_z = self.x * r20 + self.y * r21 + self.z * r22 + tz
        return Vector(new_x, new_y, new_z, name, id)

    def clone_trot(self, transform_matrix):
        return self.new_trot(transform_matrix, self.name, self.id)

    def clone_shift(self, tx, ty, tz):
        return Vector(self.x + tx, self.y + ty, self.z + tz, self.name, self.id)

    def clone_trot_shift(self, transform_matrix, tx, ty, tz):
        return self.clone_trot(transform_matrix).clone_shift(tx, ty, tz)

    def to_markdown_string(self):
        x = round(self.x, 2)
        y = round(self.y, 2)
        z = round(self.z, 2)
        markdown_string = f"{self.name}\n\n(x: {x}, y: {y}, z: {z})"
        return markdown_string