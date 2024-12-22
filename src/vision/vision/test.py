import rclpy
import cv2
from rclpy.node import Node
from .camera.Camera import Camera, ECameraNotConnected
from numpy import ndarray

class CameraPublisher(Node):
    CAMERA_NAME: str = "HD Webcam C615"
    def __init__(self):
        super().__init__("camera_publisher")
        self._camera: Camera = Camera(self.CAMERA_NAME)
        self._setup_camera()
        self._image_display_loop()
    
    def _setup_camera(self) -> None:
        try:
            self._camera.connect()
        except ECameraNotConnected as e:
            self.get_logger().warn(f"Couldn't connect: {e}")

    
    def _image_display_loop(self) -> None:
        while True:
            try:
                image: ndarray = self._camera.get_frame()
                print("Got image")
                #cv2.imshow("test", image)
                #cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f"Couldn't get frame: {e}")
                self._setup_camera()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
