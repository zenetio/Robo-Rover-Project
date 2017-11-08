---
output:
  html_document: default
  pdf_document: default
  word_document: default
---
## Project: Search and Sample Return

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg
[image4]: ./output/warped_example.jpg
[image5]: ./output/warped_threshed.jpg
[image6]: ./misc/rover_turn.jpg
[image7]: ./misc/travelled.JPG

## [Rubric]() Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. The rover and the environment  

Below is a picture where we can see the rover and, in the front of it, there is a camera and a claw to catch the objects.
The camera capture the images that are analyzed by the AI in the rover to decide the way to go, detect and avoid obstacles and, if it finds a sample near it, get the sample with claw.

### First Rubric Item

![Fig.1 - The Rober][image1]

### Rover autonomous features

#### 2. The main brain of Rover AI is the `decision.py` file where I implemented the following commands:
- forward mode
- stop mode
- turn mode
- find the path
- try to recover if blocked by obstacle
- catch the sample, if near

One of the most difficult task was make the rover turn, because we need check some telemetry status before send the 
turn command. 

![Fig.2 - Rover turning to find a valid path][image6]


#### 3. How to decide where to go using a valid path

One very important information received from transformation phase is the distance to obstacles.
Note that in the begining of decision funtion, before take any decision the code check the rover status, 
I mean, the code calculate the distance of obstacles to decide if the rover can keep going ahead or if it need take any another action. The code also assume some pre-conditions (constants), that are used as references to keep the motion more safe.

```{python}
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Default mode is forward motion
    mean_angle = np.mean(Rover.nav_angles)
    mean_dist = np.mean(Rover.nav_dists)
    angle_abs = np.absolute(mean_angle)

    sign = np.sign(mean_angle)      # get the sign
    MIN_DIST = 20
    MIN_ANG  = .15
    STEER_ANG = 10
    if sign != 0:
        STEER_ANG = 15 
```

#### 4. The perspective transformation

The perspective transformation is done using the `perspect_transform()` function. This function make use of OpenCV 2 library API. 

I used a combination of color and gradient thresholds to generate a binary image.
Below is the snippet code used to create a thresholded binary image.
The image cames from the image frame generated by camera. Using the `perspect_transform()` function in conjunction with `color_thresh()` function, the current image can be transformed, resulting the thresholded image.


```{python}
# Grab another image
warped = perspect_transform(Rover.img)
threshed = color_thresh(warped)
xpix, ypix = rover_coords(threshed)
```

The resulting thresholded image can be visualized below.

![Fig.3 - Original image][image4]

![Fig.4 - Map of navigable terrain][image5]

#### 5. Pick up samples

The most difficult part of this project was the debbug proccess because, basically, we need use print() function to display the information we need to check. Because this project make use of many packages, it is not easy to configure a debugger like PyCharm, Visual Studio or any other used by the community.
The `check_for_sample()` function is very simple and it will the rover set the "near sample" flag to pick the sample. Because of this, it will take a long time to pick up all the samples because not all times the rover will be near to sample enough to fire the "near sample" flag. In the improvement section I provide a sugestion that can improve the time to collect the samples.

---

### Discussion

### Results

The `decision()` algorithm showed to be very consistent, executing all the necessary tasks to keep the Rover in the correct way.
The figure below shows the resulting job where we can see the image from Rover perspective, the direction to take, where the Rover is located in navigable terrain and path already visited.

![Fig.6 - Resulting image][image7]

#### 1. Problems  / issues faced in the implementation of this project.

* There is no doubt for me that the most difficult part of this project was how to manage the images, for instance, overlap 2 images dealing with the channels.
* It was not easy to find out how to turn the rover. I had to use the manual mode and see what happens (telemetry values) when the rover turns. Some try and error helped find the correct way.
* I could not find a good solution to avoid the rocks in the path when the gradient thresholds process returns a valid distance value.
* I verified that check negative velocity can help a lot. It means that the rover found an obstacle, could not go ahead and is slowly runing back. So, this is time to stop and keep turning until find a new path.
* To find the sample rock I created a function that can search the image frame and return a vector where valid points refer to sample rock found. If no sample is found, then the image has no rock ahead.

## Improvements - What can be done to make it more robust?

* We can improve the search for sample rock using Gaussian distribution where we can segment out the sample rock and analyse the segmented image to see if anything was found
* We can also improve the way how the samples are collected. Using the sample location, we can guide the rover direct to the sample, stop the rover, pick the sample and then transfer the control again to the main logic that drives the rover.
* Another solution to accelerate pick up the rocks could be use a "wall crawler" mode once the rocks are located, most of the time, near the wall.

Here's a [!video [showing this project](https://youtu.be/zAKd0hIIrnk)]