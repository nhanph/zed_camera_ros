# zed_camera_ros
Ros node for zed camera without using ZED SDK, so no cuda cores are required :). Should work as a standard ros stereo camera (described here http://wiki.ros.org/stereo_image_proc).

Modified from StephMc:
- resize camera frame
- allow frame skipping