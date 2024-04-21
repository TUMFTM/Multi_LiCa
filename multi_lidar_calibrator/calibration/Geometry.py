import numpy as np
from scipy.spatial.transform import Rotation as R


class Base:
    def __init__(self, x: float, y: float, z: float):
        """
        Initialize with x, y, z translation or rotation.
        """
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return self.__class__(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return self.__class__(self.x - other.x, self.y - other.y, self.z - other.z)

    def __abs__(self):
        return self.__class__(*np.abs([self.x, self.y, self.z]))

    def __str__(self):
        return str(self.x) + " " + str(self.y) + " " + str(self.z)


class Translation(Base):
    """Handles translation in 3D space."""

    def __init__(self, x: float, y: float, z: float):
        """
        Initialize with x, y, z coordinates.

        Parameters:
        x (float): x-coordinate
        y (float): y-coordinate
        z (float): z-coordinate
        """
        super().__init__(x, y, z)

    def as_arr(self) -> np.ndarray:
        """
        Return the translation as a numpy array.

        Returns:
        np.ndarray: Array of x, y, z coordinates
        """
        return np.array([self.x, self.y, self.z], dtype=np.float64)


class Rotation(Base):
    """Handles rotation around x, y, z axes in radians."""

    def __init__(self, x: float, y: float, z: float, degrees=False):
        """
        Initialize with x, y, z rotation angles. If degrees is True, the angles are converted to radians.

        Parameters:
        x (float): Rotation around x-axis
        y (float): Rotation around y-axis
        z (float): Rotation around z-axis
        degrees (bool): If True, the angles are in degrees. Otherwise, they are in radians.
        """
        super().__init__(x, y, z)
        if degrees:
            self.x = np.deg2rad(x)
            self.y = np.deg2rad(y)
            self.z = np.deg2rad(z)

    @classmethod
    def from_quaternion(cls, quaternion: list) -> "Rotation":
        """
        Create a Rotation object from a quaternion.

        Parameters:
        quaternion (list): Quaternion to convert

        Returns:
        Rotation: A new Rotation object initialized with the Euler angles derived from the quaternion
        """
        euler = R.from_quat(quaternion).as_euler("xyz", degrees=False)
        return cls(*euler, False)  # unpack euler angles

    def as_arr(self, degrees=False) -> np.ndarray:
        """
        Return the rotation angles as a numpy array. If degrees is True, the angles are converted to degrees.

        Parameters:
        degrees (bool): If True, convert the angles to degrees

        Returns:
        np.ndarray: Array of rotation angles
        """
        if not degrees:
            return np.array([self.x, self.y, self.z], dtype=np.float64)
        else:
            return np.rad2deg(self.as_arr(False))

    def as_quaternion(self, degrees=False) -> np.ndarray:
        """
        Return the rotation as a quaternion. If degrees is True, the angles are first converted to radians.

        Parameters:
        degrees (bool): If True, convert the angles to radians before converting to a quaternion

        Returns:
         np.ndarray: Quaternion representation of the rotation
        """
        return R.from_euler("xyz", self.as_arr(degrees), degrees=degrees).as_quat()

    def __str__(self, degrees=False):
        if degrees:
            [x, y, z] = self.as_arr(True)
            return str(x) + " " + str(y) + " " + str(z)
        else:
            return str(self.x) + " " + str(self.y) + " " + str(self.z)


class TransformationMatrix:
    """
    Handles transformation matrices for 3D transformations.
    """

    def __init__(self, translation: Translation, rotation: Rotation, degrees=False):
        """
        Initialize a TransformationMatrix object.

        Args:
            translation: A Translation object representing the translation component of the transformation.
            rotation: A Rotation object representing the rotation component of the transformation.
            degrees: If True, the rotation angles are assumed to be in degrees and are converted to radians.
            If False, the rotation angles are assumed to be in radians.

        Returns:
            None
        """
        self.translation = translation
        self.rotation = rotation
        self.matrix = np.zeros((4, 4), dtype=np.float64)
        self.matrix[:-1, 3] = translation.as_arr()
        self.matrix[3] = [0, 0, 0, 1]
        self.matrix[:-1, :-1] = R.from_euler(
            "xyz", rotation.as_arr(degrees), degrees=degrees
        ).as_matrix()

    @classmethod
    def from_matrix(cls, matrix: np.ndarray):
        """
        Create a TransformationMatrix object from a 4x4 transformation matrix.

        Args:
            matrix: A 4x4 numpy array representing a transformation matrix.

        Returns:
            A TransformationMatrix object.
        """
        translation = Translation(*matrix[:3, 3])
        rotation = Rotation(*R.from_matrix(matrix[:3, :3].copy()).as_euler("xyz"))
        return cls(translation, rotation)
