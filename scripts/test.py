# This file is only for testing single code paragraph
import cv2
import imutils
import numpy as np
from imutils import paths
from distance_ObjectToCamera import find_marker, distance_to_camera
import distance_ObjectToCamera

if __name__ == "__main__":

    focal_length = distance_ObjectToCamera.get_focal_length()
    distance_30 = distance_to_camera(6.4, focal_length, 720)
    distance_50 = distance_to_camera(6.4, focal_length, 418)
    print(distance_30)
    print(distance_50)
