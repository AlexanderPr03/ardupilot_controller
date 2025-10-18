# paper_detector.py
import time
from picamera2 import Picamera2
import numpy as np
import cv2  # OpenCV is now a direct dependency

# --- Tuning Parameters for White Detection ---
# These values define the range for "white" in the HSV color space.
# You may need to adjust these based on your specific lighting conditions.
LOWER_WHITE_HSV = np.array([0, 0, 180])
UPPER_WHITE_HSV = np.array([179, 40, 255])

# The minimum area (in pixels) for a contour to be considered a valid target.
# This is crucial for filtering out small white specks of noise.
MIN_CONTOUR_AREA = 2500


class PaperDetector:
    """
    A vision system to detect the largest white rectangular object in the camera's view.
    """

    def __init__(self, width=640, height=480):
        print("Initializing Camera for White Paper Detection...")
        self.width = width
        self.height = height
        self.camera_center_x = width // 2
        self.camera_center_y = height // 2

        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (width, height), "format": "XRGB8888"})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1.0)
        print("Vision System Ready.")

    def detect(self):
        """
        Captures an image and finds the largest white rectangle.
        Returns the frame and a custom "tag-like" object if a target is found.
        """
        frame = self.picam2.capture_array()

        # Convert the captured frame to the HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Create a binary mask where white pixels are 1 and others are 0
        mask = cv2.inRange(hsv_frame, LOWER_WHITE_HSV, UPPER_WHITE_HSV)

        # Find all the contours (shapes) in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        best_target = None
        max_area = 0

        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)

                # Filter out small contours that are likely just noise
                if area > MIN_CONTOUR_AREA:
                    # Find the center of the contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # If this is the biggest valid contour we've found so far, save it
                        if area > max_area:
                            max_area = area
                            # Create a simple object that mimics the AprilTag object
                            # so our main controller doesn't need to change.
                            best_target = type('obj', (object,), {
                                'center': (cx, cy),
                                'area': area,
                                # We can fake these other properties for compatibility
                                'tag_id': 99,  # Use a fake ID for "white paper"
                                'corners': None
                            })

        return frame, best_target

    def shutdown(self):
        """Properly shuts down the camera."""
        print("Shutting down camera.")
        self.picam2.stop()
