import cv2 as cv
import numpy as np
import pyrealsense2 as realsense
from matplotlib import pyplot as plt
from PIL import Image
from pyk4a import PyK4A


class RealsenseCamera():

    def __init__(self):
        self.pipeline = realsense.pipeline()

    def get_image(self):
        self.pipeline.start()
        # get frame
        frames = self.pipeline.wait_for_frames()
        # convert rgb to np.array
        frame_rgb = frames.get_color_frame()
        image_rgb = np.asanyarray(frame_rgb.get_data())
        image_rgb = cv.cvtColor(image_rgb, cv.COLOR_RGB2BGR)
        # convert depth to np.array
        frame_depth = frames.get_depth_frame()
        image_depth = np.asanyarray(frame_depth.get_data())
        image_depth = cv.cvtColor(image_depth, cv.COLOR_RGB2BGR)
        self.pipeline.stop()
        return image_rgb, image_depth


class KinectCamera():

    def __init__(self):
        # Load camera with the default config
        self.k4aviewer = PyK4A()

    def get_image(self):
        # Get the next capturae (blocking function)
        self.k4aviewer.start()
        capture = self.k4aviewer.get_capture()
        self.k4aviewer.stop()
        img_color = capture.color / 255.0
        img_depth = capture.transformed_depth
        return img_color[:, :, 2::-1], img_depth

    def get_pointcloud(self):
        # get pointcloud
        self.k4aviewer.start()
        capture = self.k4aviewer.get_capture()
        self.k4aviewer.stop()
        return capture.transformed_depth_point_cloud / 1000.0

    def save_calibration(self, path):
        self.k4aviewer.start()
        self.k4aviewer.save_calibration_json(path)
        self.k4aviewer.stop()

    def show_image(self):
        # get image
        img_color, img_depth = self.get_image()
        # Display with pyplot
        plt.imshow(img_color)  # BGRA to RGB
        plt.show()
        # Display with pyplot
        plt.imshow(img_depth)
        plt.show()


if __name__ == '__main__':
    camera = RealsenseCamera()
    image_rgb, image_depth = camera.get_image()
    plt.imshow(image_rgb)
    plt.show()
    plt.imshow(image_depth)
    plt.show()
    print(np.shape(image_rgb))
    print(np.shape(image_depth))
