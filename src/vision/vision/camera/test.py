from Camera import Camera, EReadFrameError, ECameraNotConnected
from get_camera_info import cameras_information
import cv2

def main():
    cameras = cameras_information()
    for camera_name, camera_index in cameras.items():
        print(f"Camera name: {camera_name}, Camera index: {camera_index}")

    cam = Camera(2, "Logitech")

    
    while True:
        
        if not cam.is_connected():
            try:
                cam.connect()
            except ECameraNotConnected as e:
                print(e)

        if not cam.is_connected():
            continue
        try:
            frame = cam.get_frame()
        except EReadFrameError as e:
            print(e)
            continue
            
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    main()