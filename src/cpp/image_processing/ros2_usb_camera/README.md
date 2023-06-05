# ros2_usb_camera

Simple ROS 2 driver node for USB monocular cameras compatible with the `Video4Linux` APIs. Based on `image_transport`, `camera_calibration` and `OpenCV`.

## Features

- `CameraInfo` topic.
- `CompressedImage` topics for both color and rectified-color images.
- `Image` topics for both color and rectified-color images.
- Hardware enable service, based on `std_srvs/srv/SetBool`.
- Supports namespace and node name remappings, in order to run different cameras with multiple instances of the node.
- ROS 2 component compilation and installation.
- Optimized memory handling.
- Supports Nvidia CUDA hardware and the OpenCV GPU module.
- High-resolution, thread-based camera sampling.
- Offers both reliable and best-effort transmissions, configurable via node parameters.
- `calibrator` node as standalone ROS 2 executable, to perform nonstandard calibration routines.

## Usage

The code compiles to both a standalone application executable and a ROS 2 component, and both can be run easily. There is also a launch file for the standalone application.

The CMake configuration automatically detects if a compatible CUDA Toolkit installation is available, and if OpenCV has been built with CUDA support. In such case, parts of the code that perform processing such as rectification are replaced with `cv::cuda` API calls. For this to work, OpenCV must have been built from source with CUDA support, which requires a working installation of the CUDA Toolkit and a compatible Nvidia GPU.

Once the node is started, the video capture device will be disabled by default. To toggle it, send a request on the `~/enable_camera` service specifying either `True` or `False` in the `data` field.

You can use `RViz` to display the frames being streamed:

- topic `Reliability Policy` must be set to what the corresponding node parameter has been set to;
- `Fixed Frame` in `Global Options` must be set to what the corresponding node parameter has been set to.

### Node Parameters

Configuration files for node parameters can be found in `config`, with some standard default settings. They can be customized, or overridden from command line or launch files.

- `base_topic_name`: base transmission topic name for `image_transport` publishers.
- `best_effort_qos`: toggles unreliable but faster transmissions.
- `brightness`: camera brightness level (hardware-dependent).
- `camera_calibration_file`: camera calibration YAML file URL.
- `camera_id`: ID of the video capture device to open.
- `exposure`: camera exposure time (hardware-dependent).
- `fps`: camera capture rate, defaults to `20`.
- `frame_id`: transform frame_id of the camera, defaults to `map`.
- `image_height`: image height, defaults to `480`.
- `image_width`: image width, defaults to `640`.
- `is_flipped`: toggles vertical image flipping.
- `wb_temperature`: white balance temperature (hardware-dependent).

Keep in mind that hardware-dependent parameters are particularly tricky: they might not be supported, have unusual or even completely different ranges, and require some black magic to be correctly set up. What you see in this code was done to work with some cameras we had at the time, so be prepared to change many things if you want to act on camera hardware settings.

### Camera Calibration

The necessary parameters and camera intrinsics can be acquired from a standard calibration procedure. You can write your own routine for this, e.g. with `OpenCV`, or use the `camera_calibration cameracalibrator` tool as documented [here](https://navigation.ros.org/tutorials/docs/camera_calibration.html).

### Calibrator Application

If you need to perform some additional calibration steps, you can encode them in the `CalibratorNode`, part of the `calibrator` standalone application. The node structure is straightforward: parameters are defined and used in the same way, as well as launch files, and everything happens in an Image callback. You can set the name of the topic and the transport you prefer, and define your additional routine inside the `image_transport` subscriber callback. You can also set new parameters as you like.

Currently, as an example, the node computes the camera focal distance in pixels as part of the [Pinhole Camera Model](https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/), using an [ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) marker of known size placed at a known distance from the camera.

## Copyright

Copyright © 2022 Intelligent Systems Lab
