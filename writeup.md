# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

I first implemented an EKF, complete with the predict/measurement steps and the covariance calculations. Then track management was implemented, which registers multi-target tracking, confirmation and deletion when out of view. Target/measurement association was then implemented, which associates the lidar and camera measurement with the registered targets. Finally, the camera nonlinear model was added to the measurement model, after which the EKF performs measurement update with camera as well.

The last step was the most difficult since minor bugs, which were masked in the previous steps, began to manifest.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 

There is a decreased number of tentative ghost tracks with camera fusion. There is also a small decrease in the mean error in target tracking.

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?

The results become more sensitive to the gating function due to the higher number of detected objects in camera. Disagreements in the measurement devices may also need to be rationalized.

### 4. Can you think of ways to improve your tracking results in the future?

Determining object yaw from camera should produce a more precise velocity measurement update. 