# This file is only for testing single code paragraph
import cv2
import imutils
import numpy as np
from imutils import paths
import distance_angle_Object

if __name__ == "__main__":

    focal_length = distance_angle_Object.get_focal_length()
    distance_30 = distance_angle_Object.distance_to_camera(6.4, focal_length, 720)
    distance_50 = distance_to_camera(6.4, focal_length, 418)
    print(distance_30)
    print(distance_50)
