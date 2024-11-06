import cv2
import pyzed.sl as sl
import numpy as np
import math
from ultralytics import YOLO
import datetime

class CameraNode:
    def __init__(self, weight):
        # Initialize YOLO model
        self.model = YOLO(weight)

        # Initialize ZED camera
        self.zed = sl.Camera()

        # Camera initialization parameters
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD1080
        self.init_params.camera_fps = 10

        # Open ZED camera
        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            print("Failed to open camera")
            return

        # Runtime parameters
        self.runtime_parameters = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.point_cloud = sl.Mat()

    def update(self):
        # Grab the next frame from the ZED camera
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            img = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGR2RGB)

            # Run YOLO model on the image
            results = self.model(img)
            boxes = results[0].boxes.xyxy.tolist()

            # Initialize lists to store coordinates and labels
            xlist = []
            ylist = []
            zlist = []
            labels = []

            # Extract 3D coordinates and labels for each detected object
            for xyxy in boxes:
                x1, y1, x2, y2 = map(int, xyxy[:4])
                x, y = (x1 + x2) // 2, (y1 + y2) // 2  # Get the center of the box

                err, point_cloud_value = self.point_cloud.get_value(x, y)
                if err == sl.ERROR_CODE.SUCCESS:
                    x_coord, y_coord, z_coord = point_cloud_value[:3]
                    xlist.append(x_coord)
                    ylist.append(y_coord)
                    zlist.append(z_coord)

                    # Loop through all labels detected by YOLO in the current frame
                    for i, box in enumerate(results[0].boxes):
                        label = results[0].names[box.cls]
                        labels.append(label)
                        print(f"Label: {label}, X: {x_coord:.2f}cm, Y: {y_coord:.2f}cm, Z: {z_coord:.2f}cm")

            # Save data to file if boxes are detected
            if len(boxes) > 0:
                filename = "output.txt"

                # Open file in write mode and overwrite content
                with open(filename, "w") as f:
                    time = datetime.datetime.now().strftime("%Y-%m-%d %H-%M-%S-%f")
                    f.write(f"Time: {time}\n")
                    for i in range(len(xlist)):
                        # Write the 3D coordinates and labels
                        f.write(f"{zlist[i]} {-xlist[i]} {labels[i]}\n")
                    f.write(f"Time: {time}\n")


if __name__ == "__main__":
    try:
        # Make sure to specify the path to your YOLO weights file
        weight_path = "best111.pt"
        
        # Initialize the CameraNode with the YOLO weights
        camera_node = CameraNode(weight_path)
        
        # Run the update method in a loop
        while True:
            camera_node.update()

    except KeyboardInterrupt:
        print("Process interrupted. Exiting...")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        # Properly close the ZED camera when the program exits
        if 'camera_node' in locals() and camera_node.zed.is_open():
            print("Closing ZED camera.")
            camera_node.zed.close()

