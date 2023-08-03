# Visualization
## Saving results
To save results, set `debug=True` in the `__init__()` of `ORBVisualOdometry` in `src/visual_odometry.py` file. 

At each iteration (pair of images processed), this will save the updated position in the file `position.txt`. This will also save the pair of images as one image with a portion of the matches, coordinate system representation and current transition (translation and rotation between the two images).

**WARNING:** This will save a pair of image at each iteration which can lead to a very big `/results` folder. For example if `time_between_frames_s = 0.1`, running the odometry script for 3 minutes will have 1800 images saved.

## Process results

Run:
```
python3 viz/visualization.py
```
to plot a bird's-eye view of the trajectory, matches and transition at each step. 
This will also create a `trajectory.gif` like this one:

![](https://github.com/Rob1in/viam_visual_odometry/blob/main/img/trajectory.gif)

## Configurate .gif

```
python3 viz/visualization.py -h
usage: visualization.py [-h] [--input_folder INPUT_FOLDER] [--input_position_file INPUT_POSITION_FILE] [--output_gif_file OUTPUT_GIF_FILE] [--frame_duration FRAME_DURATION] [--loop_count LOOP_COUNT]

Process position.txt. Plot bird's-eye view of the trajectory. Show matches for each pair of images. Create gif from JPEG images.

optional arguments:
  -h, --help            show this help message and exit
  --input_folder INPUT_FOLDER
                        Path to the folder containing JPEG images and positions file.
  --input_position_file INPUT_POSITION_FILE
                        Name of the .txt storing positions in the input folder.
  --output_gif_file OUTPUT_GIF_FILE
                        Name of the output gif file.
  --frame_duration FRAME_DURATION
                        Duration of each frame in milliseconds.
  --loop_count LOOP_COUNT
                        Number of loops for the gif (0 for infinite loop).```