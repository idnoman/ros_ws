import cv2
import numpy as np
from overrides import override

try:
    # for ROS2
    from .ICamera import ICamera, ECameraNotConnected, EReadFrameError
    from .get_camera_info import cameras_information
except ImportError:
    # for standalone testing
    from ICamera import ICamera, ECameraNotConnected, EReadFrameError
    from get_camera_info import cameras_information


class Camera(ICamera):
    _camera: cv2.VideoCapture
    _connected: bool
    _camera_index: int
    _camera_name: str

    @override
    def __init__(self, camera_name: str):
        self._connected = False
        self._camera_name = camera_name

    def __del__(self):
        if self._connected:
            self.disconnect()

    @override
    def connect(self) -> None:
        if self._connected:
            return
        try:
            info: dict = cameras_information()
        except:
            raise ECameraNotConnected("Error getting camera information")
        try:
            index = info[self._camera_name]
        except KeyError:
            raise ECameraNotConnected(f"Camera {self._camera_name} not found")
        self._camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
        # self._connected = self._camera.isOpened()
        self._connected = True

    @override
    def disconnect(self) -> None:
        self._camera.release()
        self._connected = False

    @override
    def is_connected(self) -> bool:
        return self._connected

    @override
    def get_frame(self) -> np.ndarray:
        if not self._connected:
            raise ECameraNotConnected("Camera is not connected")

        ret, frame = self._camera.read()
        if not ret:
            self.disconnect()
            raise EReadFrameError("Error reading frame")

        return frame
