

# The function to calculate the focallength
def get_focal_length():
    # initialize the known distance from the camera to the known object, unit in mm.
    KNOWN_distance_real = 20.0
    KNOWN_length_real = 10.0
    KNOWN_length_in_image = 2
    # formula of calculating the focallength
    focal_length = (KNOWN_length_in_image * KNOWN_distance_real) / KNOWN_length_real
    return focal_length
