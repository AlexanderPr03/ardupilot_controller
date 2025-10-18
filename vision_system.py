# vision_system.py
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

        # Initialize the Pi Camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (width, height), "format": "XRGB8888"})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1.0)  # Allow camera to warm up

        # Initialize the AprilTag detector
        self.tag_detector = Detector(
            families="tag36h11",  # A common and robust tag family
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )
        print("Vision System Ready.")

    def detect(self):
        """
        Captures an image, detects AprilTags, and returns tag info.
        Returns the processed frame and a list of detected tags.
        """
        # Capture a frame and convert to numpy array
        frame = self.picam2.capture_array()

        # Convert to grayscale for detection
        gray_frame = np.dot(frame[..., :3], [0.2989, 0.5870, 0.1140]).astype(np.uint8)

        # Detect tags
        tags = self.tag_detector.detect(gray_frame)

        # Filter for the main tag if multiple are detected
        # For now, we'll just use the first one found.
        detected_tag = None
        if tags:
            detected_tag = tags[0]

        return frame, detected_tag

    def shutdown(self):
        """Properly shuts down the camera."""
        print("Shutting down camera.")
        self.picam2.stop()
