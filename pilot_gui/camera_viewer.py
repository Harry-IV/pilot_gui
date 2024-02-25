# Author: Harry O'Hagin '26
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import cv2
import yaml
from ament_index_python.packages import get_package_share_directory

# Finds the path for the share directory where the config will be located
package_share_directory = get_package_share_directory("pilot_gui")

class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")
        self.log = self.get_logger()
        self.vid_capture = None
        config_path = package_share_directory + "/config/camera_ips.yaml"
        with open(config_path,'r') as f:
            # Dictionary storing the IPs of the known cameras
            self.known_cameras = yaml.load(f, Loader=yaml.SafeLoader)
        self.working_cameras = {}
        # Checks if all the cameras are working
        self.check_cameras()
        # stops the program if there are no working cameras
        if(self.working_cameras == {}):
            self.log.error("NO WORKING CAMERAS!")
            exit()
        # Makes sure that it doesn't change to the same camera
        self.vid_capture = None
        self.ip = self.working_cameras[list(self.working_cameras.keys())[0]]
        # Changes the vid_capture to "Top" because check_cameras() leaves vid_capture set to "Bottom"
        self.vid_capture = cv2.VideoCapture(f"http://{self.ip}:5000")
        # Purpose of current_cam is for the change_camera() function
        self.current_cam = list(self.working_cameras.keys())[0]
        # Creates a subscriber that changes the camera when the correct button on the joystick is pressed 
        # Note: The callback only runs when it detects new input from the joystick.
        self.joy_sub = self.create_subscription(Joy,"joy",self.change_camera,qos_profile=10)
        # Callback for displaying the camera feed
        # Note: To lower latency, the callback for displaying the feed is set to 200 times per second
        self.log.info("LAUNCHING STREAM!")
        self.create_timer(1/200.0,self.display_frame)
 
    def check_cameras(self):
        for camera in self.known_cameras.keys():
            working = False
            ip = self.known_cameras[camera]
            self.log.info(f"Checking {camera} camera @ {ip}")
            try:
                self.vid_capture = cv2.VideoCapture(f"http://{ip}:5000")
                self.current_cam = camera
                for i in range(3):
                    # result will be true if it captures a successful frame
                    result, frame = self.vid_capture.read()
                    if result:
                        working = True
                        self.working_cameras[camera] = self.known_cameras[camera]
                        self.log.info(f"{camera} camera is working!")
                        break           
                    # if it didn't capture a successful frame, it will try another 2 times
                    else:
                        time.sleep(0.5)
            # cv2 doesn't throw error so finally was used instead of except
            finally:
                if(not working):
                    self.log.warn(f"Check if the {camera} camera's stream script is running, if the pi's ethernet cord is disconnected, or if it's the wrong ip.")
        
    def display_frame(self):
        result, frame = self.vid_capture.read()
        if(result):
            # Both of these lines are ESSENTIAL for displaying the frame onto the screen
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1) 
   
    def change_camera(self,joy):
        # Button indexes are mapped for a PS4 controller (indexes will varry for other setups)
        # Checks if correct button is pressed and that it doesn't match the current camera being streamed
        if(joy.buttons[2] == 1 and not self.current_cam == "Top" and "Top" in self.working_cameras):
            self.log.info("Switching to top camera")
            self.ip = self.known_cameras["Top"]
            self.vid_capture = cv2.VideoCapture(f"http://{self.ip}:5000")
            self.current_cam = "Top"
        if(joy.buttons[0] == 1 and not self.current_cam == "Bottom" and "Bottom" in self.working_cameras):
            self.log.info("Switching to bottom camera")
            self.ip = self.known_cameras["Bottom"]
            self.vid_capture = cv2.VideoCapture(f"http://{self.ip}:5000")
            self.current_cam = "Bottom"
    
def main(args=None):
   rclpy.init(args=args)
   camera_viewer = CameraViewer()
   rclpy.spin(camera_viewer)
   camera_viewer.vid_capture.release()
   cv2.destroyAllWindows()
   camera_viewer.destroy_node()
   rclpy.shutdown()
 
if __name__ == "__main__":
   main()