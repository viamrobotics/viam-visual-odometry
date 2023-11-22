# Monocular Visual Odometry Movement Sensor
[Viam module](https://docs.viam.com/extend/modular-resources/) for monocular visual odometry implemented as a movement sensor.

![https://github.com/Rob1in/viam_visual_odometry/blob/main/img/trajectory.gif](https://github.com/viamrobotics/viam-visual-odometry/blob/main/img/trajectory.gif?raw=true)

## Requirements

You must have a calibrated [camera](https://docs.viam.com/components/camera/) configured on your robot.

This module implements two methods of the [movement sensor API](https://docs.viam.com/components/movement-sensor/#api):
  * `GetLinearVelocity()`
  * `GetAngularVelocity()`

Please note that `GetLinearVelocity()` returns an estimation of the instantaneous linear velocity **without scale factor**. Hence, units should not be trusted and `GetLinearVelocity()` should serve as direction estimation.

## Build and Run

To use this module, follow these instructions to [add a module from the Viam Registry](https://docs.viam.com/registry/configure/#add-a-modular-resource-from-the-viam-registry) and select the `viam:visual_odometry:opencv_orb` model from the [`monocular-visual-odometry` module](https://app.viam.com/module/viam/agilex-limo).

## Configure your Monocular Visual Odometry Movement Sensor

> [!NOTE]  
> Before configuring your movement sensor, you must [create a robot](https://docs.viam.com/manage/fleet/robots/#add-a-new-robot).

Navigate to the **Config** tab of your robot’s page in [the Viam app](https://app.viam.com/). Click on the **Components** subtab and click **Create component**. Select the `movement_sensor` type, then select the `visual_odometry:opencv_orb` model. Enter a name for your movement sensor and click **Create**.

On the new component panel, copy and paste the following attribute template into your movement sensor’s **Attributes** box. 

```json
{
  "camera_name": "<your-camera-name>",
  "time_between_frames_s": <time_seconds>,
  "lowe_ratio_threshold": <lowe_ratio_threshold>
}
```

> [!NOTE]  
> For more information, see [Configure a Robot](https://docs.viam.com/manage/configuration/).

### Attributes

The following attributes are available for `visual_odometry:opencv_orb` movement sensors:

| Name | Type | Inclusion | Default | Description |
| ---- | ---- | --------- | --------| ------------ |
| `camera_name` | string | **Required** | | Name of the camera you want to use to infer motion. |
| `time_between_frames_s` | float | Optional | `0.1` | Target time between two successive frames given in seconds. Depending on the inference time and the time to get an image, the sleeping time after each inference will be auto-tuned to reach this target. Also, if the time between two successive frames is 5x larger than `time_between_frames_s`, another frame will be requested. This value depends on the speed of your system. |
|`orb_n_features`| int | Optional | `10000` | Maximum number of features to retain. |
|`orb_edge_threshold`| int | Optional | `31` | Size of the border where the features are not detected. It should roughly match the `orb_patch_size` attribute.  |
|`orb_patch_size`| int | Optional | `31` | Size of the patch used by the oriented BRIEF descriptor.|
|`orb_n_levels`| int | Optional | `8` |Number of pyramid levels.|
|`orb_first_level`| int | Optional | `0` |Level of pyramid to put source image to.|
|`orb_fast_threshold`| int | Optional | `20` | Fast threshold. |
|`orb_scale_factor`| float | Optional | `1.2` | Pyramid decimation ratio, greater than 1. |
|`orb_WTA_K`| int | Optional | `2` | Number of points that produce each element of the oriented BRIEF descriptor. |
|`matcher`| string | Optional | `"flann"` | Either `"flann"` for [FLANN based matcher](https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html) or `"BF"` for brute force matcher. The FLANN matcher will look for the two best matches using the KNN method so Lowe's ratio test can be performed afterward. The [brute force matcher](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) uses Hamming norm. |
|`lowe_ratio_threshold`| float | Optional | `0.8` | Threshold value to check if the best match is significantly better than the second best match. This value will not be used if the brute force matcher is chosen. |
| `ransac_prob` | float | Optional | `0.99` | Probability to find a subset without outliers in it. Defines the number of iterations to filter the outliers. The number of iterations is roughly given by $k = \frac{\log(1-p)}{\log(1-w^n)}$, where $n$ is the number of points, $w$ the ratio of inliers to total points.|
| `ransac_threshold_px` | float | Optional | `2` | Maximum error to be classified as an inlier.|

See the [ORB openCV documentation](https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html) for more details.

### Example configuration (with camera) 

```json
{
  "modules": [
    {
      "type": "registry",
      "name": "viam_monocular-visual-odometry",
      "module_id": "viam:monocular-visual-odometry",
      "version": "0.0.8"
    }
  ],
  "components": [
    {
      "name": "cam",
      "type": "camera",
      "model": "webcam",
      "attributes": {
        "height_px": 720,
        "width_px": 1280,
        "intrinsic_parameters": {
          "ppx": 446,
          "ppy": 585,
          "fx": 1055,
          "fy": 1209
        },
        "distortion_parameters": {
          "rk3": -0.03443,
          "tp1": 0.01364798,
          "tp2": -0.0107569,
          "rk1": -0.1621,
          "rk2": 0.13632
        }
      },
      "depends_on": []
    },
    {
      "namespace": "rdk",
      "model": "viam:visual_odometry:opencv_orb",
      "attributes": {
        "camera_name": "cam", 
        "time_between_frames_s": 0.2, 
        "lowe_ratio_threshold": 0.75
      },
      "depends_on": [],
      "name": "visual_odometry1",
      "type": "movement_sensor"
    }
  ]
}

```

The camera **needs** to have intrinsics parameters. You can follow these [instructions](https://github.com/viam-labs/camera-calibration/tree/main) to calibrate your camera.

### Local Installation

#### Option 1: `poetry`

```
cd viam-visual-odometry
poetry install
```
 In `run.sh`, uncomment the line:

  ```
  #exec poetry run python -m src.main $@
  ```

  and remove the line:
  ```
  exec python3 -m src.main $@
  ```


#### Option 2 : `pip install`

```
pip install -r requirements.txt
```

## Deeper dive

The module works as follow:
  1. Having a previous image with keypoints, request a new image. 
  2. Detect ORB keypoints in the new image. 
  3. Find matching keypoints in the two image using KNN or brute force matcher.
  4. Filter matches (with Lowe's and RANSAC) and compute essential matrix.
  5. Decompose essential matrix using chirality constraint. 
  6. Retrieve linear and angular velocities from the previous decomposition. 

### Matcher

### Filtering operations
### Coordinate system
The coordinate system used is as follows.
X is pointing to the right of the camera, Y down, and Z forward.
<p align="center">
 <img src="https://github.com/Rob1in/viam-visual-odometry/blob/main/img/coordinate_system.png" width=35%, height=35%>
 </p>

### Angular velocity calculation from rotation matrix

From the rotation matrix, we first compute the Euler angles as the serie of [intrinsic rotations](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html) (= attached to the moving body) $ZXZ$, giving angles $\phi$,  $\theta$ and $\psi$.\
\
Angular velocities about the **final** $x,y,z$ coor­dinate system is given by:

$\omega_x = \dot{\phi} \sin\theta \sin\psi + \dot{\theta} \cos\psi$\
$\omega_y = \dot{\phi} \sin\theta \cos\psi - \dot{\theta} \sin\psi$\
$\omega_z = \dot{\phi} \cos\theta + \dot{\psi}$

For more details see [here](https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/resources/mit16_07f09_lec29/).

## References

## Troubleshooting



