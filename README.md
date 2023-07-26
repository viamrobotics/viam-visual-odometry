# Monocular Visual Odometry
[Viam module](https://docs.viam.com/extend/modular-resources/) for monocular visual odometry implemented as a movement sensor.

![](https://github.com/Rob1in/viam_visual_odometry/blob/main/img/trajectory.gif)

## Getting started
All you need is a calibrated camera.

This module implements two methods [movement sensor API](https://docs.viam.com/components/movement-sensor/#api):
  * `GetLinearVelocity`
  * `GetAngularVelocity`


Please note that GetLinearVelocity returns an estimation of the instantaneous linear velocity **without scale factor**. Hence, units should not be trusted and GetLinearVelocity should serve as direction estimation.

## Config
### Sample config 
```json
{
  "modules": [
    {
      "name": "my-odometry",
      "executable_path": "/path/to/run.sh"
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
        "video_path": ""
      },
      "depends_on": []
    },
    {
      "namespace": "rdk",
      "model": "viam:opencv:visual_odometry_orb",
      "attributes": {
        "camera_name": "cam"
      },
      "depends_on": [],
      "name": "visual_odometry1",
      "type": "movement_sensor"
    }
  ]
}

```

### Parameters description
The following attributes are available to configure your Visual odometry module:

| Name | Type | Inclusion | Default | Description |
| ---- | ---- | --------- | --------| ------------ |
| `camera_name` | string | **Required** | | Camera name to be used for infering the motion. |
| `time_between_frames_s` | float | Optional | `0.1` | Target time between two successive frames given in seconds. Depending on the inference time and the time to get an image, the sleeping time after each inference will be auto-tuned to reach this target. Also, if the time between two successive frame is 5x bigger than `time_between_frames_s`, another frame will be requested. |
|`orb_n_features`| int | Optional | `10000` | Maximum number of features to retain. |
|`orb_edge_threshold`| int | Optional | `31` | Size of the border where the features are not detected. It should roughly match the patchSize parameter.  |
|`orb_patch_size`| int | Optional | `31` | Size of the patch used by the oriented BRIEF descriptor.|
|`orb_n_levels`| int | Optional | `8` |Number of pyramid levels.|
|`orb_first_level`| float | Optional | `0` |Level of pyramid to put source image to.|
|`orb_fast_threshold`| int | Optional | `20` | Fast threshold. |
|`orb_scale_factor`| float | Optional | `1.2` | Pyramid decimation ratio, greater than 1. |
|`orb_WTA_K`| int | Optional | `2` | Number of points that produce each element of the oriented BRIEF descriptor. |
|`matcher`| string | Optional | `"flann"` | Either `"flann"` for [FLANN based matcher](https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html)  or  `"BF"` for brute force matcher. FLANN matcher will look for the two best matches using the KNN method so Lowe's ratio test can be performed afterward. [Brute force matcher](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) uses Hamming norm. |
|`lowe_ratio_threshold`| float | Optional | `0.8` | Threshold value to check if the best match is significantly better than the second best match. This value will not be used if brute force matcher is chosen. |
| `ransac_prob` | float | Optional | `0.99` | Probabilty to find a subset without outliers in it. Defines the number of iterations to filter the outliers. The number of iterations is roughly given by $k = \frac{\log(1-p)}{\log(1-w^n)}$, where $n$ is the number of points, $w$ the ratio of inliers to total points.|
| `ransac_threshold_px` | float | Optional | `0.5` | Maximum error to be classified as inlier.|

See [ORB openCV documentation](https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html) for more details.

## Parameters contribution on computation time

## References

## Troubleshooting



