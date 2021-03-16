import numpy as np
import cv2 as cv
from pyk4a import PyK4A
from matplotlib import pyplot as plt
from PIL import Image


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
    camera = KinectCamera()
    # _, depth = camera.get_image()
    # depth = np.array(Image.open('mask.png'))
    # print(depth[0,0], depth[360,640], np.shape(depth))
    # depth = (depth > 128)
    # print(depth[0,0], depth[360,640])
    # depth = Image.fromarray(depth)
    # depth.save('mask_1.png')
    camera.save_calibration('calibration.json')
    # print(np.shape(camera.get_pointcloud()))