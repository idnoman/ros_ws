import cv2
import numpy as np


class ECameraNotConnected(Exception):
    pass

class EReadFrameError(Exception):
    pass


class ICamera:
    
    def __init__(self, camera_index: int, camera_name: str):
        pass

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def is_connected(self) -> bool:
        pass

    def get_frame(self) -> np.ndarray:
        pass