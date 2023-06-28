import numpy as np
# import rospy
import math
import basic_move
from apriltag import get_apriltag
# from object_localization import ObjecT
from object_localization import get_objects
# from motors_waveshare import MotorControllerWaveshare

def find_nearest_cube(cubes):
    """
    Return the nearest cube in one detection

    Args:
        cubes (list of object): the cubes, which are found in one detection

    Returns:
        nearest_cube (ObjecT): the nearest cube in one detection
    """
    min_distance = cubes[0].distance
    min_index = 0
    for i in range(len(cubes)):
        if cubes[i].distance < min_distance:
            min_distance = cubes[i].distance
            min_index = i

    return cubes[min_index]

def turning_detection(excepted_cubes):
    """
    Turning in the current position and try to find the next cube at the same time, which not lies on the base

    Args:
        excepted_cubes (list of ObjecT): the cubes, which have been pushed to the corresponding base.

    Returns:
        next_cube (ObjecT): the next cube need to be pushed
    """
    _, orientation = get_apriltag()
    current_direction = orientation[2]
    # every pi / 4 do a detection
    for n in range(8):
        if n != 0:
            basic_move.turn_to_direction(jetbot_motor, current_direction + np.pi / 4 * n)
        objects = get_objects()
        for i in range(len(objects)):
            if objects[i] in excepted_cubes:
                objects.pop(i)
        if objects != []:
            next_cube = find_nearest_cube(objects)
            return next_cube

    return None

def go_to_next_cube(next_cube, excepted_cubes):
    """
    Go to the next cube and get its position again to improve accuracy, call the function when next_cube.distance > 0.2

    Args:
        next_cube (ObjecT): the next_cube
        excepted_cubes (list of ObjecT): the cubes, which have been pushed to the corresponding base.

    Returns:
        cube_will_be_pushed (ObjecT): the cube will be pushed
    """
    if (next_cube.distance <= 0.2):
        return next_cube

    distance = next_cube.distance - 0.2
    duration = distance / 0.1
    basic_move.linear_motion_with_desired_time(jetbot_motor, next_cube.position, duration)
    objects = get_objects()
    for i in range(len(objects)):
        if objects[i] in excepted_cubes:
            objects.pop(i)
    if objects != []:
        cube_will_be_pushed = find_nearest_cube(objects)
        return cube_will_be_pushed

def go_to_start_push_position(base_position, object_position):
    """
    Go to the position, on there the next push should be started

    Args:
        base_position ([float, float]): the position of the corresponding base
        object_position ([float, float]): the position of the object need to be pushed
    """
    DISTANCE_TO_OBJECT = 0.15
    vec = [object_position[0] - base_position[0], object_position[1] - base_position[1]]
    magnitude = math.sqrt(vec[0]**2 + vec[1]**2)
    uVec = [vec[0] / magnitude, vec[1] / magnitude]
    p_x = (DISTANCE_TO_OBJECT * uVec[0]) + object_position[0]
    p_y = (DISTANCE_TO_OBJECT * uVec[1]) + object_position[1]
    start_push_position = [p_x, p_y]

    position, _ = get_apriltag()
    completed = (abs(position[0] - start_push_position[0])) < 0.03 and (abs(position[1] - start_push_position[1]) < 0.03)
    while not completed:
        continue_flag = False
        position, _ = get_apriltag()
        objects = get_objects()
        while objects != []
            obstacle = find_nearest_cube(objects)
            distance_to_line = basic_move.distance_point_line(position, start_push_position, obstacle.position)
            if distance_to_line < 0.08:
                approach_obstacle(obstacle.position)
                my_obstacle_avoid(jetbot_motor) #TODO
                continue_flag = True
                break
        if continue_flag:
            continue
        basic_move.linear_motion_with_desired_time(jetbot_motor, start_push_position, 3)
        completed = (abs(position[0] - start_push_position[0])) < 0.03 and (abs(position[1] - start_push_position[1]) < 0.03)

def approach_obstacle(obstacle_position):
    """
    Approach the obstacle for avoiding

    Args:
        obstacle_position ([float, float]): the position of the obstacle need to avoid
    """
    DISTANCE_TO_OBSTACLE = 0.10
    position, _ = get_apriltag()
    vec = [obstacle_position[0] - position[0], obstacle_position[1] - position[1]]
    magnitude = math.sqrt(vec[0]**2 + vec[1]**2)
    uVec = [vec[0] / magnitude, vec[1] / magnitude]
    p_x = (-DISTANCE_TO_OBSTACLE * uVec[0]) + object_position[0]
    p_y = (-DISTANCE_TO_OBSTACLE * uVec[1]) + object_position[1]
    p = [p_x, p_y]
    basic_move.linear_motion_with_desired_time(jetbot_motor, p)

def my_obstacle_avoid(jetbot_motor):
    """
    Choose the side automatically. Only for obstacles not in the corner #TODO
    """
    position, orientation = get_apriltag()
    position = position[0:2]
    current_direction = orientation[2]
    if position[0] < 0.15:
        # left side up
        if (current_direction > 0 and current_direction < (np.pi / 6)) or (current_direction > (np.pi / 6 * 11)):
            if position[1] > 1.47:
                # basic_move.backwards_distance(jetbot_motor, 0.15)
                # basic_move.turn_to_direction(jetbot_motor, np.pi / 2 * 3)
                return
            else:
                basic_move.avoid_obstacle(jetbot_motor, 1)
                return
        # left side down
        elif current_direction > (np.pi / 6 * 5) and current_direction < (np.pi / 6 * 7):
            if position[1] < 0.15:
                # basic_move.stop(jetbot_motor)
                # basic_move.turn_to_direction(jetbot_motor, np.pi / 2 * 3)
                return
            else:
                basic_move.avoid_obstacle(jetbot_motor, 0)
                return
    elif position[0] > 1.47:
        # right side up
        if (current_direction > 0 and current_direction < (np.pi / 6)) or (current_direction > (np.pi / 6 * 11)):
            if position[1] > 1.47:
                # basic_move.backwards_distance(jetbot_motor, 0.15)
                # basic_move.turn_to_direction(jetbot_motor, np.pi / 2)
                return
            else:
                basic_move.avoid_obstacle(jetbot_motor, 0)
                return
        # right side down
        elif current_direction > (np.pi / 6 * 5) and current_direction < (np.pi / 6 * 7):
            if position[1] < 0.15:
                # basic_move.stop(jetbot_motor)
                # basic_move.turn_to_direction(jetbot_motor, np.pi / 2)
                return
            else:
                basic_move.avoid_obstacle(jetbot_motor, 1)
                return

    if position[1] < 0.15:
        # under side left
        if (current_direction > (np.pi / 3)) and (current_direction < (np.pi / 3 * 2)):
            if position[0] < 0.15:
                # basic_move.stop(jetbot_motor)
                # basic_move.turn_to_direction(jetbot_motor, 0)
                return
            else:
                basic_move.avoid_obstacle(jetbot_motor, 1)
                return
        # under side right
        elif (current_direction > (np.pi / 3 * 4)) and (current_direction < (np.pi / 3 * 5)):
            if position[0] > 1.47:
                # basic_move.stop(jetbot_motor)
                # basic_move.turn_to_direction(jetbot_motor, 0)
                return
            else:
                basic_move.avoid_obstacle(jetbot_motor, 0)
                return
    elif position[1] > 1.47:
        # up side left
        if (current_direction > (np.pi / 3)) and (current_direction < (np.pi / 3 * 2)):
            if position[0] < 0.15:
                # basic_move.stop(jetbot_motor)
                # basic_move.turn_to_direction(jetbot_motor, np.pi)
                return
            else:
                basic_move.avoid_obstacle(jetbot_motor, 0)
                return
        # up side right
        elif (current_direction > (np.pi / 3 * 4)) and (current_direction < (np.pi / 3 * 5)):
            if position[0] > 1.47:
                # basic_move.stop(jetbot_motor)
                # basic_move.turn_to_direction(jetbot_motor, np.pi)
                return
            else:
                basic_move.avoid_obstacle(jetbot_motor, 1)
                return

    basic_move.avoid_obstacle(jetbot_motor, 0)

def push_cube(base_position, cube):
    """
    Push a cube to the corresponding base position, start from start_push_position

    Args:
        base_position ([float, float]): the position of the corresponding base
        cube (ObjecT): the cube, which will be pushed
    """
    cube_distance = cube.distance
    duration_to_cube = cube_distance / 0.1
    basic_move.linear_motion_with_desired_time(jetbot_motor, base_position, duration_to_cube)

    position, _ = get_apriltag()
    completed = (abs(position[0] - base_position[0])) < 0.05 and (abs(position[1] - base_position[1]) < 0.05)
    while not completed:
        continue_flag = False
        position, _ = get_apriltag()
        objects = get_objects()
        if cube in objects:
            objects.remove(cube)
        while objects != []
            obstacle = find_nearest_cube(objects)
            distance_to_line = basic_move.distance_point_line(position, base_position, obstacle.position)
            if distance_to_line < 0.08:
                approach_obstacle(obstacle.position)
                my_obstacle_avoid(jetbot_motor) #TODO
                continue_flag = True
                break
        if continue_flag:
            continue
        basic_move.linear_motion_with_desired_time(jetbot_motor, base_position, 5) # linear_motion_with_desired_time_with_cube
        completed = (abs(position[0] - base_position[0])) < 0.05 and (abs(position[1] - base_position[1]) < 0.05)

# push is just like linear_motion
# after push, go backward 10cm
# before linear_motion always turning_detection, when find a cube, push it instead of going back to path or any other motion.

def go_back_to_path(path):
    """
    After pushing a cube, go back to center or end of the predefined path and search for next_cube at the same time

    Args:
        path_start ([float, float]): the start point of the path
        path_end ([float, float]): the end point of the path

    Returns:
        arrive_end (bool): arrive the path_end or not
    """
    position, _ = get_apriltag()
    distance_to_path = basic_move.distance_point_line(path.start, path.end, position)
    if distance_to_path < 0.1:
        arrive_end = (abs(position[0] - path.end[0])) < 0.05 and (abs(position[1] - path.end[1]) < 0.05)
        return arrive_end

    path_center = [(path.start[0] + path.end[0]) / 2, (path.start[1] + path.end[1]) / 2]
    distance_to_end = math.sqrt((position[0] - path.end[0]) ** 2 + (position[1] - path.end[1]) ** 2)
    if distance_to_end > 0.2:
        # go back to path_center
        completed = (abs(position[0] - path_center[0])) < 0.05 and (abs(position[1] - path_center[1]) < 0.05)
        while not completed:
            continue_flag = False
            position, _ = get_apriltag()
            objects = get_objects()
            while objects != []
                obstacle = find_nearest_cube(objects)
                distance_to_line = basic_move.distance_point_line(position, path_center, obstacle.position)
                if distance_to_line < 0.08:
                    approach_obstacle(obstacle.position)
                    my_obstacle_avoid(jetbot_motor) #TODO
                    continue_flag = True
                    break
            if continue_flag:
                continue
            basic_move.linear_motion_with_desired_time(jetbot_motor, path_center, 5)
            completed = (abs(position[0] - path_center[0])) < 0.05 and (abs(position[1] - path_center[1]) < 0.05)
        arrive_end = False
        return arrive_end
    else:
        # go back to path_end
        completed = (abs(position[0] - path.end[0])) < 0.05 and (abs(position[1] - path.end[1]) < 0.05)
        while not completed:
            continue_flag = False
            position, _ = get_apriltag()
            objects = get_objects()
            while objects != []
                obstacle = find_nearest_cube(objects)
                distance_to_line = basic_move.distance_point_line(position, path.end, obstacle.position)
                if distance_to_line < 0.08:
                    approach_obstacle(obstacle.position)
                    my_obstacle_avoid(jetbot_motor) #TODO
                    continue_flag = True
                    break
            if continue_flag:
                continue
            basic_move.linear_motion_with_desired_time(jetbot_motor, path.end, 3)
            completed = (abs(position[0] - path.end[0])) < 0.05 and (abs(position[1] - path.end[1]) < 0.05)
        arrive_end = True
        return arrive_end

def follow_path(path_end):
    """
    Call the function after go_back_to_path and let Jetbot follow the path.
    
    Args:
        path_end ([float, float]): the end point of the path

    Returns:
        arrive_end (bool): arrive the path_end or not
    """
    position, _ = get_apriltag()
    basic_move.linear_motion_with_desired_time(jetbot_motor, path_end, 3)
    arrive_end = (abs(position[0] - path_end[0])) < 0.05 and (abs(position[1] - path_end[1]) < 0.05)
    return arrive_end


# maybe don't need
# def is_finished(excepted_cubes):
#     """
#     Determine if all the cubes has been pushed to corresponding base
#
#     Args:
#         excepted_cubes (list of ObjecT): the cubes, which have been pushed to corresponding base
#
#     Returns:
#         bool
#     """
#     if len(excepted_cubes) == 6:
#         return True

class Path:
    """
    Class of Path

    Attributes:
        path_start ([float, float])
        path_end ([float, float])
    """
    def __init__(self, start, end):
        self.start = start
        self.end = end

def define_paths():
    """
    Define some paths. Jetbot should follow these paths to search cubes and push them

    Returns:
        paths (list of Path): the paths
    """
    paths = []
    path0 = Path([0.2, 0.2], [1.465, 1.465])
    paths.append(path0)
    path1 = Path([1.465, 1.465], [1.465, 0.2])
    paths.append(path1)
    path2 = Path([1.465, 0.2], [0.2, 1.465])
    paths.append(path2)
    path3 = Path([0.2, 1.465], [1.465, 1.465])
    paths.append(path3)
    path4 = Path([1.465, 1.465], [0.2, 0.2])
    paths.append(path4)
    path5 = Path([0.2, 0.2], [0.2, 1.465])
    paths.append(path5)
    path6 = Path([0.2, 1.465], [1.465, 0.2])
    paths.append(path6)
    path7 = Path([1.465, 0.2], [0.2, 0.2])
    paths.append(path7)
    return paths

# base positions
START_POSITION = []
RED_BASE_POSITION = []
GREEN_BASE_POSITION = []
BLUE_BASE_POSITION = []
YELLOW_BASE_POSITION = []
PURPLE_BASE_POSITION = []
ORANGE_BASE_POSITION = []
def get_base_position(cube_name):
    """
    Get the corresponding base_position according to cube's name
    
    Args:
        cube_name (string): the name of cube

    Returns:
        base_position ([float, float]): the position of corresponding base
    """
    if cube_name == "cube_red":
        return RED_BASE_POSITION
    elif cube_name == "cube_green":
        return GREEN_BASE_POSITION
    elif cube_name == "cube_blue":
        return BLUE_BASE_POSITION
    elif cube_name == "cube_yellow":
        return YELLOW_BASE_POSITION
    elif cube_name == "cube_purple":
        return PURPLE_BASE_POSITION
    elif cube_name == "cube_orange":
        return ORANGE_BASE_POSITION

def main():
    # initialization
    # visited_positions = [] #linear_motion with desired time = 3s, every 3 seconds save a position to visited_positions
    excepted_cubes = [] # cubes which have been pushed to the corresponding base
    paths = define_paths()

    # move to paths[0].start
    basic_move.linear_motion_with_desired_time(jetbot_motor, paths[0].start, 3)

    for i in range(len(paths)):
        arrive_end = False
        while not arrive_end:
            next_cube = turning_detection(excepted_cubes)
            if next_cube is not None:
                base_position = get_base_position(next_cube.name)
                cube_will_be_pushed = go_to_next_cube(next_cube, excepted_cubes)
                go_to_start_push_position(base_position, cube_will_be_pushed.position)
                push_cube(base_position, cube_will_be_pushed)
                excepted_cubes.append(cube_will_be_pushed)
                if len(excepted_cubes) == 6:
                    basic_move.linear_motion(jetbot_motor, START_POSITION)
                    return
                basic_move.backwards_distance(jetbot_motor, 0.1)
            else:
                arrive_end = go_back_to_path(paths[i])
                if not arrive_end:
                    arrive_end = follow_path(paths[i].end)


if __name__ == "__main__":
    main()
