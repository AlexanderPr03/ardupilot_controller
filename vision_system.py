import time
from picamera2 import Picamera2
from pupil_apriltags import Detector
import numpy as np

class AprilTagDetector:
    def __init__(self, width=640, height=480):
        print("Initializing Camera and Vision System...")
        self.width = width
        self.height = height
        self.camera_center_x = width // 2
        self.camera_center_y = height // 2

        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (width, height), "format": "XRGB8888"})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1.0)

        self.tag_detector = Detector(families="tag36h11", nthreads=1)
        print("Vision System Ready.")

    def detect(self, target_tag_id=0):
        """
        Captures an image and searches for a specific AprilTag ID.
        """
        frame = self.picam2.capture_array()
        gray_frame = np.dot(frame[..., :3], [0.2989, 0.5870, 0.1140]).astype(np.uint8)
        tags = self.tag_detector.detect(gray_frame)

        # CORRECT: Loop through all detected tags to find our specific target
        detected_tag = None
        for tag in tags:
            if tag.tag_id == target_tag_id:
                detected_tag = tag
                break # Stop searching once we find our target

        return frame, detected_tag

    def shutdown(self):
        """Properly shuts down the camera."""
        print("Shutting down camera.")
        self.picam2.stop()
