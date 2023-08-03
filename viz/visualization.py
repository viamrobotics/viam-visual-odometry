import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import cv2
import os
from PIL import Image
import re
import argparse


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

def plot_and_save_projection_XZ(arrays_list, path_to_results_folder):
    "save the sere"
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
        plt.savefig(path_to_results_folder + f"/traj_{int(id[i])}.jpg")
    # plt.show()
    return id
    

def insert_trajectory_on_odometry(id, path_to_results_folder):
    for i in id:
        try:
            matches = cv2.imread(path_to_results_folder+f"/match{int(i)}.jpg")
            traj = cv2.imread(path_to_results_folder + f"/traj_{int(i)}.jpg")
        except:
            continue
        try:
            height, width = matches.shape[:2]
            image2_resized = cv2.resize(traj, (width//4, height//2))
            # Get the dimensions of the first image

            # Get the dimensions of the second image (after resizing)
            h2, w2 = image2_resized.shape[:2]
            
            # Insert the second image into the first image
            matches[height-h2:height, width-w2:width] = image2_resized

            # Save the final combined image
            cv2.imwrite( path_to_results_folder + f"/match{int(i)}.jpg", matches)  
        except:
            
            continue      


def sort_jpeg_names(jpeg_files):
    def extract_numeric_part(filename):
        # Extract the numeric part 'n' from the filename
        match = re.match(r'match(\d+)\.jpg', filename)
        if match:
            return int(match.group(1))
        
        return 0  # Return 0 if the format doesn't match

    return sorted(jpeg_files, key=extract_numeric_part)
    
def create_gif_from_jpegs(jpeg_folder, gif_filename, duration=100, loop=0, prefix_filter="match", images_to_skip=3, reduce_factor=2):
    jpeg_files = [file for file in os.listdir(jpeg_folder) if ((file.lower().endswith('.jpg') or file.lower().endswith('.jpeg')) and file.lower().startswith(prefix_filter))]
    jpeg_files = sort_jpeg_names(jpeg_files=jpeg_files)

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
    images[0].save(jpeg_folder+"/"+gif_filename, save_all=True, append_images=images[1:], duration=duration, loop=loop)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create gif from JPEG images.")
    parser.add_argument("--input_folder", type=str, default="./results", help="Path to the folder containing JPEG images and positions file.")
    parser.add_argument("--input_position_file", type=str, default="position.txt", help="Path to .txt storing poses")
    parser.add_argument("--output_gif_file", type=str, default="trajectory.gif", help="Path to the output gif file.")
    parser.add_argument("--frame_duration", type=int, default=100, help="Duration of each frame in milliseconds.")
    parser.add_argument("--loop_count", type=int, default=0, help="Number of loops for the gif (0 for infinite loop).")
    args = parser.parse_args()
    position_file = args.input_folder + "/" + args.input_position_file
    arrays = read_txt_file(position_file)
    # plot_3d_points(arrays)
    ids = plot_and_save_projection_XZ(arrays, args.input_folder)
    insert_trajectory_on_odometry(ids, args.input_folder)

    create_gif_from_jpegs(args.input_folder, args.output_gif_file, duration=args.frame_duration, loop=args.loop_count)
    