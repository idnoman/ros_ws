"""
You may wonder what actually hapens in here. We use the v4l2-ctl command to get the list of video devices.
The output of this command is a string that contains the name of the camera and the index of the camera.

example output:

GENERAL WEBCAM: GENERAL WEBCAM (usb-0000:05:00.4-1):
        /dev/video2
        /dev/video3
        /dev/media1

HP HD Camera: HP HD Camera (usb-0000:05:00.4-4):
        /dev/video0
        /dev/video1
        /dev/media0


We use the output to extract the name of the camera and the index of the camera.
In first line we treat the string from beginning to first ' (' as the name of the camera.

In next lines we get linux devices that are cameras. Sometimes you get one, sometimes two or more.
The first device under camera name with 'video' in its name is the one that actually provides the video stream. 
We extract the index of the camera from the first such device under the camera name. Other devices are used for other purposes.
We skip cameras that do not provide a 'video' device.

We return a dictionary that maps camera name to the index of the camera.

example output:

{'GENERAL WEBCAM': 2, 'HP HD Camera': 0}


INFO: the function actually meant to be used is 
    cameras_information() that returns a dictionary of cameras.

"""

import subprocess

def _get_camera_info_from_system() -> str:
    # Get the list of video devices using v4l2-ctl
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True, check=True)
        output = result.stdout
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        return []

    return output

def _extract_camera_information(shell_output: str) -> dict:
    cameras = {}
    lines = shell_output.splitlines()
    found_camera = False
    for line in lines:
        # Skip empty lines
        if line == "":
            found_camera = False
            continue

        if not found_camera and not '/dev' in line:
           words = line.split(' (')
           camera_name = words[0].strip()
           found_camera = True
           continue

        if found_camera and '/dev/video' in line:
            words = line.split('/dev/video')
            index = words[-1].strip()
            cameras[camera_name] = int(index)
            found_camera = False  
            continue 

    return cameras

def cameras_information() -> dict:
    """
    This function returns dictionary, where:
        - keys are camera names (str)
        - values are camera indexes (int)

    example output:
    {'GENERAL WEBCAM': 2, 'HP HD Camera': 0}

    Camera indexes are ready to use with cv2.VideoCapture(index)
    
    NOTE: for know function wasn't tested with case when there are more than one camera with the same name.
    """
    output = _get_camera_info_from_system()
    cameras = _extract_camera_information(output)
    return cameras


if __name__ == "__main__":
    cameras = cameras_information()
    print(cameras)
    
