import numpy as np
from scipy.spatial.transform import Rotation
import cv2
import math
from PIL import Image


def get_camera_matrix(focal_length_x, focal_length_y, principal_point_x, principal_point_y):
    camera_matrix = np.array([[focal_length_x, 0, principal_point_x],
                              [0, focal_length_y, principal_point_y],
                              [0, 0, 1]])
    return camera_matrix


def get_distort_param(rk1, rk2, rk3, tp1, tp2):
    distortion_params = np.array([rk1, rk2, rk3, tp1, tp2])
    return distortion_params


def opcv2rpy(vec):
    R = Rotation.from_euler('XZ', [90, 90], degrees=True).as_matrix()
    return np.matmul(R, vec)


def rotation_matrix_to_euler(rot_matrix, seq = "XYZ"):
    return Rotation.from_matrix(rot_matrix).as_euler(seq, degrees=True)


def rotation_matrix_to_euler_smallest_norm(matrix):
    sequences = ['XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX']
    euler_angles_smallest_norm = None
    smallest_norm = float('inf')

    for seq in sequences:
        r = Rotation.from_matrix(matrix)
        angles = r.as_euler(seq, degrees=True)
        norm = np.linalg.norm(angles)

        if norm < smallest_norm:
            smallest_norm = norm
            euler_angles_smallest_norm = (seq, angles)

    return euler_angles_smallest_norm



def draw_perspective_axis(image):
    
    # Get the image dimensions
    width, height = image.shape[:2]
    print(f"width {width}, height : {height}")
    # Set the length of the arrow lines
    arrow_length = min(width, height) // 10
    
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = .8
    thickness = 2
   

    # Define the colors for the axes
    x_color = (0, 0, 255)   # Blue color for X-axis
    y_color = (0, 255, 0)   # Green color for Y-axis
    z_color = (255, 0, 0)   # Red color for Z-axis
    
    # origin = (height//2, width//2)
    origin = (int(height - 1.5*arrow_length), int(1.5*arrow_length))
    origin = (100,150)

    # # Draw X-axis
    endpoint = (int(origin[0]+arrow_length), origin[1])
    cv2.arrowedLine(image, origin, endpoint, x_color, 2)
    cv2.putText(image, "X", (endpoint[0]+arrow_length//6, endpoint[1]), font, font_scale, x_color, thickness)
    
    
    # # Draw Y-axis 
    endpoint = (int(origin[0]), int(origin[1]+arrow_length))
    cv2.arrowedLine(image, origin, endpoint, y_color, 2)
    cv2.putText(image, "Y", (endpoint[0], endpoint[1]+arrow_length//3), font, font_scale, y_color, thickness)
    
    
    # Draz Z-axis
    endpoint = (int(origin[0]+0.866*arrow_length), int(origin[1]-0.5*arrow_length))
    cv2.arrowedLine(image, origin, endpoint, z_color, 2)
    cv2.putText(image, "Z", (endpoint[0]+arrow_length//7, endpoint[1]-arrow_length//7), font, font_scale, z_color, thickness)


    return image

def draw_values(final_img, x, y, z):
    text = f" X: {round(x,2)}, Y:{round(y,2)}, Z:{round(z,2)}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1.0
    color = 255  # White color (since the image is grayscale)
    thickness = 4

    # Specify the position of the text 
    position = (100, 50)  # (x, y) coordinates

    # Write the text on the grayscale images
    cv2.putText(final_img, text, position, font, font_scale, color, thickness)
    

def euler_to_angular_rate(phi, theta, psi, dt):
    if dt ==0:
        return 0,0,0
    # from here:
    # https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/resources/mit16_07f09_lec29/
    w_x = phi / dt * math.sin(np.deg2rad(theta)) * math.sin(np.deg2rad(psi)) + theta / dt * math.cos(
        np.deg2rad(psi))
    w_y = phi / dt * math.sin(np.deg2rad(theta)) * math.cos(np.deg2rad(psi)) - theta / dt * math.sin(
        np.deg2rad(psi))
    w_z = phi / dt * math.cos(np.deg2rad(theta)) + psi / dt
    
    return w_x, w_y, w_z

def rotation_matrix_to_euler_smallest_norm(matrix):

    sequences = ['XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX']
    euler_angles_smallest_norm = None
    smallest_norm = float('inf')

    for seq in sequences:
        r = Rotation.from_matrix(matrix)
        angles = r.as_euler(seq, degrees=True)
        norm = np.linalg.norm(angles)
        print(f"FOR SEQUENCE : {seq}, angles are {angles}")
        if norm < smallest_norm:
            smallest_norm = norm
            euler_angles_smallest_norm = angles, seq

    return euler_angles_smallest_norm, seq




def save_numpy_array_to_file_on_new_line(array, file_path):
    print("SAVING")
    if not isinstance(array, np.ndarray):
        raise ValueError("Input should be a valid numpy array.")

    try:
        with open(file_path, 'a') as file:
            np.savetxt(file, [array], delimiter=',', newline='\n', fmt='%s')
    except IOError as e:
        print(f"An error occurred while saving the array to '{file_path}': {e}")



def check_norm(R):
    phi, theta, psi = Rotation.from_matrix(R).as_euler(seq="XYZ", degrees=True)
    if np.linalg.norm(np.array([phi, theta, psi]))>100:
        return np.eye(3)
    
    return R