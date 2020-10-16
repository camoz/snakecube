#!/usr/bin/env python3


def multiply(matrix, vector):
    """Return the matrix product of a matrix and a vector.

    Args:
        matrix (list(list(int))): A matrix of size m*n.
        vector (list(int)): A vector of size n.

    Returns:
        Vector3D: A vector of size m (the matrix product of the two arguments).

    """
    result = []
    for row in matrix:
        assert len(row) == len(vector)
        result.append(sum([a*b for (a, b) in zip(row, vector)]))
    return Vector3D.from_list(result)


class Vector3D:
    """A simple implementation of 3D vectors.

    Args:
        x (int): x coordinate of the vector.
        y (int): y coordinate of the vector.
        z (int): z coordinate of the vector.

    Attributes:
        x (int): x coordinate of the vector.
        y (int): y coordinate of the vector.
        z (int): z coordinate of the vector.

    """

    __slots__ = ['x', 'y', 'z', '_pos']

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f'Vector3D({self.x!r}, {self.y!r}, {self.z!r})'

    def __getitem__(self, index):
        return [self.x, self.y, self.z][index]

    def __len__(self):
        return 3

    def __iter__(self):
        self._pos = 0
        return self

    def __next__(self):
        if self._pos >= self.__len__():
            raise StopIteration
        pos = self._pos
        self._pos += 1
        return self[pos]

    def __add__(self, other):
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __rmul__(self, other):
        return Vector3D(self.x * other, self.y * other, self.z * other)

    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y and self.z == other.z)

    def to_list(self):
        """Return the list of coordinates of this object.

        Returns:
            list(int): The coordinates of this object.

        """
        return [self.x, self.y, self.z]

    @classmethod
    def from_list(cls, lst):
        """Create a Vector3D object from a plain list of coordinates.

        Args:
            lst (list(int)): A list of coordinates.

        Returns:
            Vector3D: The new vector created from the list of coordinates.

        """
        return cls(lst[0], lst[1], lst[2])


baseVectors = [Vector3D(1,0,0), Vector3D(0,1,0), Vector3D(0,0,1)]
"""The three base vectors in 3D space."""

baseDirections = [Vector3D(1,0,0), Vector3D(0,1,0), Vector3D(0,0,1),
                  Vector3D(-1,0,0), Vector3D(0,-1,0), Vector3D(0,0,-1)]
"""The three base vectors in 3D space together with their inverse."""

# rotation matrices for 90 degree rotation of 3D *base* vectors
rotx = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
roty = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
rotz = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]

# mirror matrices for mirroring a vector at the yz/zx/xy plane
mirror_yz = [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]
mirror_zx = [[1, 0, 0], [0, -1, 0], [0, 0, 1]]
mirror_xy = [[1, 0, 0], [0, 1, 0], [0, 0, -1]]


def main():
    return 0


if __name__ == '__main__':
    main()


