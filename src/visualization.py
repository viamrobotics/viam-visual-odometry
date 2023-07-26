import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import Normalize
import cv2
import os
from PIL import Image
import re

def read_txt_file(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    arrays_list = []
    for line in lines:
        values = line.strip().split(',')
        array = np.array([float(val) for val in values]).reshape((4, 1))
        arrays_list.append(array)

    return arrays_list

def plot_3d_points(arrays_list):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for array in arrays_list:
        x, y, z = array.flatten()
        ax.scatter(x, y, z)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

def plot_and_save_projection_XZ(arrays_list):
    fig, ax = plt.subplots()

    # Extract X and Z coordinates from the arrays
    id = [array[0][0] for array in arrays_list]
    X = [array[1][0] for array in arrays_list]
    Z = [array[3][0] for array in arrays_list]

    # Create a colormap for the line based on the range of Z values
    print(f"There are {len(Z)} values")
    norm = Normalize(vmin=0, vmax=len(Z))
    cmap = plt.cm.get_cmap('viridis')

    # Plot the line with progressively changing colors
    for i in range(len(X) - 1):
        ax.plot([X[i], X[i+1]], [Z[i], Z[i+1]], color=cmap(norm(i)), linewidth=3)

        ax.set_xlabel('X Label')
        ax.set_ylabel('Z Label')
        plt.axis('equal')
        plt.savefig(f"../results/traj_{int(id[i])}.jpg")
    # plt.show()
    return id
    

def insert_trajectory_on_odometry(id):
    for i in id:
        print(f"doing {i}")
        try:
            pic = cv2.imread(f"/Users/robinin/visual_odometry/opencv_visual_odometry/results/old{int(i)}.jpg")
            traj = cv2.imread(f"/Users/robinin/visual_odometry/opencv_visual_odometry/results/traj_{int(i)}.jpg")
        except:
            continue
        try:
            height, width = pic.shape[:2]
            image2_resized = cv2.resize(traj, (width//4, height//2))
            # Get the dimensions of the first image

            # Get the dimensions of the second image (after resizing)
            h2, w2 = image2_resized.shape[:2]
            
            # Insert the second image into the first imagew
            pic[height-h2:height, width-w2:width] = image2_resized

            # Save the final combined image
            cv2.imwrite(f"/Users/robinin/visual_odometry/opencv_visual_odometry/results/old{int(i)}.jpg", pic)  
        except:
            
            continue      


def sort_jpeg_names(jpeg_files):
    def extract_numeric_part(filename):
        # Extract the numeric part 'n' from the filename
        match = re.match(r'old(\d+)\.jpg', filename)
        if match:
            return int(match.group(1))
        
        return 0  # Return 0 if the format doesn't match

    return sorted(jpeg_files, key=extract_numeric_part)
    
def create_gif_from_jpegs(jpeg_folder, gif_filename, duration=100, loop=0, prefix_filter="old", images_to_skip=5, reduce_factor=2):
    jpeg_files = [file for file in os.listdir(jpeg_folder) if ((file.lower().endswith('.jpg') or file.lower().endswith('.jpeg')) and file.lower().startswith(prefix_filter))]
    jpeg_files = sort_jpeg_names(jpeg_files=jpeg_files)
    print(jpeg_files)

    if not jpeg_files:
        raise ValueError(f"No JPEG files found in the specified folder with names starting with '{prefix_filter}'.")

    images = []
    for i, jpeg_file in enumerate(jpeg_files):
        if i % (images_to_skip + 1) == 0:  # Skip images_to_skip images between each included image
            file_path = os.path.join(jpeg_folder, jpeg_file)
            image = Image.open(file_path)
            new_width = image.width // reduce_factor
            new_height = image.height // reduce_factor
            resized_image = image.resize((new_width, new_height), Image.LANCZOS)
            images.append(resized_image)

    # Save the images as a GIF
    images[0].save(gif_filename, save_all=True, append_images=images[1:], duration=duration, loop=loop)



if __name__ == "__main__":
    # txt_file = "../results/position.txt"  # Replace with the name of your txt file
    # arrays = read_txt_file(txt_file)
    # # plot_3d_points(arrays)
    # id = plot_and_save_projection_XZ(arrays)
    # insert_trajectory_on_odometry(id)
    
    
    # Create gif
    input_folder = "../results"
    output_gif = "../results/trajectory.gif"
    frame_duration = 100  # In milliseconds (100ms = 0.1 seconds)
    loop_count = 0  # 0 means infinite loop, otherwise provide a positive integer for the loop count

    create_gif_from_jpegs(input_folder, output_gif, duration=frame_duration, loop=loop_count)
    
    
    
    



