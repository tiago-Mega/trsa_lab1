# TRSA Lab 1 - 2D Image Processing with ROS 2 and OpenCV

Package: `trsa_lab1` | ROS 2 Jazzy | Ubuntu 24.04 | Python (rclpy, OpenCV, cv_bridge)

Group: `<77847>`

## 1. Overview

The package implements a small vision pipeline:

```
camera_driver --/camera/image_raw --> camera_rectifier --/camera/image_rect --> image_convert --/camera/image_processed --> image_reader
                       |                                        
                       |                                       
                       |                                                              
                       +--(or raw, if no matching calibration)-----------------------------> /object_detection/alignment--> 
                                                                                             /object_detection/image
                                                                                             /object_detection/count
camera_calibration_pub --/camera/image_raw--> cameracalibrator (produces ost.yaml)
```

## 2. Nodes and topics

| Node (executable) | Subscribes | Publishes | Purpose |
|---|---|---|---|
| `camera_driver` | - | `/camera/image_raw` | Reads a video file (or webcam) with `cv2.VideoCapture` and publishes frames (~33 Hz). Parameter `loop` (default `true`). |
| `image_reader` | `/camera/image_raw`, `/camera/image_processed` | - | Shows both streams with `cv2.imshow`. The raw window closes if no frames arrive for 1 s. |
| `camera_calibration_pub` | - | `/camera/image_raw` | Replays the staff-provided chessboard images for calibration. Parameters: `images_dir`, `publish_rate`, `frame_id`. |
| `camera_rectifier` | `/camera/image_raw` | `/camera/image_rect` | Loads `calibration/ost.yaml` into a `CameraInfo`, builds a `PinholeCameraModel` and calls `rectifyImage`. |
| `image_convert` | `/camera/image_rect` | `/camera/image_processed` | BGR -> grayscale -> Gaussian blur (21x21, sigma 0) -> Canny. All 8-bit (`mono8`). |
| `object_detection` | `/camera/image_raw` (or `/camera/image_rect`) | `/object_detection/image` (Image), `/object_detection/count` (Int32), `/object_detection/alignment` (String) | Colour + shape detection and alignment assistant (bonus). |

## 3. Build and setup

```bash
cd ~/ros2_ws/src
# copy / unzip trsa_lab1 here
sudo apt install ros-jazzy-camera-calibration ros-jazzy-image-geometry ros-jazzy-cv-bridge
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`setup.py` must install the data files (video, calibration images and `ost.yaml`) and register one console script per node above. Package dependencies: `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`, `image_geometry`, `python3-opencv`, `python3-numpy`, `python3-yaml`.

## 4. Part 1 - Driver, calibration, rectification

1. **Camera driver and viewer**
   ```bash
   ros2 run trsa_lab1 camera_driver
   ros2 run trsa_lab1 image_reader      # or RViz -> Add -> Image -> /camera/image_raw
   ```
2. **Calibration** (chessboard 8x6 inner corners, 0.0745 m squares)
   ```bash
   ros2 run camera_calibration cameracalibrator \
      --size 8x6 \
      --square 0.0745 \
      --no-service-check \
      --ros-args \
      -r image:=/camera/image_raw
   ros2 run trsa_lab1 camera_calibration_pub
   ros2 run trsa_lab1 camera_driver
   ```
   When CALIBRATE becomes active, click it (scale = 0 to crop black borders), then SAVE. Move the resulting `ost.yaml` and `.txt` into `calibration/`, rebuild.
   ```
3. **Rectification**
   ```bash
   ros2 run trsa_lab1 camera_driver
   ros2 run trsa_lab1 camera_rectifier
   ```
   View `/camera/image_raw` and `/camera/image_rect` side by side in RViz.

Note: the test video must have the same resolution as the images used for calibration.

## 5. Part 2 - Image processing

```bash
ros2 run trsa_lab1 camera_driver
ros2 run trsa_lab1 camera_rectifier
ros2 run trsa_lab1 image_convert
ros2 run trsa_lab1 image_reader
```

`image_convert` converts the rectified image to grayscale, applies `cv2.GaussianBlur` with a 21x21 kernel and sigma 0, and then `cv2.Canny`. The output is published as `mono8` on `/camera/image_processed`.

## 6. Final exercise - Object detection

Assigned target (edit to match your group): **`<COLOUR> <SHAPE>`**, default in code: `blue rectangle`.

### 6.1 Run

```bash
ros2 run trsa_lab1 camera_driver
ros2 run trsa_lab1 object_detection --ros-args -p target_color:=blue -p target_shape:=rectangle
ros2 topic echo /object_detection/count
ros2 topic echo /object_detection/alignment
```

Webcam reproduction: in `camera_driver`, replace the video path with `cv2.VideoCapture(0)` (device index of your webcam). Use a plain contrasting background, good even lighting, separated objects facing the camera. Because the webcam has no matching calibration, the detector uses `/camera/image_raw`. It never uses `/camera/image_processed`, because grayscale and Canny remove the colour information.

### 6.2 Processing pipeline

1. **Gaussian blur 7x7** - reduces sensor noise so that the colour mask has fewer speckles.
2. **BGR -> HSV** - hue separates colour from brightness, so segmentation is more robust to lighting than RGB thresholds.
3. **`cv2.inRange` colour segmentation** - HSV ranges (OpenCV scale, H 0-179):

   | Colour | H | S | V |
   |---|---|---|---|
   | blue | 95-135 | 100-255 | 50-255 |
   | green | 35-85 | 70-255 | 40-255 |
   | red | 0-10 and 170-180 (two bands, since red wraps around) | 90-255 | 50-255 |
   | yellow | 18-32 | 100-255 | 100-255 |

   A minimum saturation rejects white, grey and black regions; a minimum value rejects dark areas.
4. **Morphology (5x5 rectangle, close then open)** - close fills holes inside objects, open removes small noise blobs.
5. **`cv2.findContours` (external)** - one contour per object.
6. **Area filter** - contours smaller than `min_contour_area` (1200 px) are ignored.
7. **Shape classification** - `approxPolyDP` with epsilon = 4% of the perimeter:
   - 3 vertices -> triangle.
   - 4 vertices -> rectangle if contour area / `minAreaRect` area > 0.75, otherwise quadrilateral.
   - otherwise circle if circularity 4*pi*A/P^2 > 0.75.
   - else unknown.
   Using `minAreaRect` makes the result independent of rotation in the image plane.
8. **Validation** - an object is a valid target only if its colour mask AND its shape both match. Objects matching one criterion are neither drawn nor counted.
9. **Output** - rotated bounding box, centroid (from image moments), label `<colour> <shape>`, total target count overlay.

### 6.3 Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `target_color` | `blue` | `blue`, `green`, `red` or `yellow` |
| `target_shape` | `rectangle` | `triangle`, `rectangle`, `circle` |
| `tol_x` | 0.05 | horizontal tolerance, fraction of image width |
| `tol_y` | 0.05 | vertical tolerance, fraction of image height |
| `min_contour_area` | 1200.0 | minimum contour area in pixels |

Justification: the 1200 px area removes noise but keeps objects of roughly 35x35 px or larger; the 0.75 rectangularity and circularity thresholds tolerate imperfect hand-cut paper shapes; 4% epsilon smooths contour noise without merging corners. Adjust to your scene if needed.

### 6.4 Published topics

- `/object_detection/image` (`sensor_msgs/Image`, bgr8) - annotated image.
- `/object_detection/count` (`std_msgs/Int32`) - valid targets in the current frame (0 if none, not cumulative).
- `/object_detection/alignment` (`std_msgs/String`) - `ALIGNED`, `LEFT`, `RIGHT`, `ABOVE`, `BELOW` or `NO_TARGET`.

### 6.5 Bonus - Visual alignment assistant

The target position is the image centre; the alignment region is a rectangle of +/- `tol_x` * width and +/- `tol_y` * height around it (drawn in cyan). The valid target with the largest area is selected and highlighted with a magenta dot. Its offsets `dx = cx - target_x`, `dy = cy - target_y` are displayed. State rules:

- Inside both tolerances -> `ALIGNED`.
- Otherwise compare normalised offsets `|dx|/tol_px_x` and `|dy|/tol_px_y`; the larger one decides, with ties going to horizontal.
- `dx > 0` -> `RIGHT`, `dx < 0` -> `LEFT`, `dy > 0` -> `BELOW`, `dy < 0` -> `ABOVE` (positions in the image, not robot motion).
- No valid target -> `NO_TARGET`.

## 7. Demonstration

Video: `<LINK>`. The scene contains: valid targets (several), a distractor with the correct colour but wrong shape, a distractor with the correct shape but wrong colour. The video shows: multiple valid detections, both distractors rejected, count 0 and `NO_TARGET` when targets are removed, objects moved and rotated, and all alignment states.

## 8. Limitations

- Fixed HSV ranges depend on lighting; strong shadows, reflections or a background with similar hue can break segmentation.
- Overlapping or touching objects merge into one contour and may be misclassified.
- Strong perspective or partially visible objects change the vertex count; squares and rectangles are not distinguished.
- Shape classification is by vertex count/circularity, so shapes like ellipses or rounded rectangles may be misclassified.
- Calibration quality depends on the number and variety of chessboard poses; edges can be over-rectified with few poses.
- Only four colours and three shapes are supported.

## 9. Package layout

```
trsa_lab1/
  trsa_lab1/ camera_driver.py image_reader.py camera_calibration_pub.py
             camera_rectifier.py image_convert.py object_detection.py
  calibration/ ost.yaml ost.txt images/
  video/ <test videos>
  package.xml setup.py setup.cfg README.md
```
