import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh_above(img, rgb_thresh=(160, 160, 160)):
    """
    Create an array of zeros same xy size as img, but single channel
    """
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                & (img[:, :, 1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Identify pixels below the threshold
# Threshold of RGB < 160 does a nice job of identifying ground pixels only
def color_thresh_below(img, rgb_thresh=(160, 160, 160)):
    """
    Create an array of zeros same xy size as img, but single channel
    """
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be below all three threshold values in RGB
    # below_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] < rgb_thresh[0]) \
                & (img[:, :, 1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    #x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    #y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

# Define a function to find the color rock in the image
def find_rock_color(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all thre threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    offset1 = .3
    offset2 = .3
    offset3 = .9
    offsetR = np.int(offset1 * rgb_thresh[0])
    offsetG = np.int(offset2 * rgb_thresh[1])
    offsetB = np.int(offset3 * rgb_thresh[2])
    R = rgb_thresh[0]
    G = rgb_thresh[1]
    B = rgb_thresh[2]
    Rval = ((img[:,:,0] > (R-offsetR)) & (img[:,:,0] < (R+offsetR)))
    Gval = ((img[:,:,1] > (G-offsetG)) & (img[:,:,1] < (G+offsetG)))
    Bval = ((img[:,:,2] > (B-offsetB)) & (img[:,:,2] < (B+offsetB)))
    #print(Rval)
    found_sample = Rval & Gval & Bval
    # Index the array of zeros with the boolean array and set to 1
    color_select[found_sample] = 1
    # Return the binary image with Region Of Interest
    return color_select

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    img_size = (Rover.img.shape[1], Rover.img.shape[0])
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    bottom_offset = 6
    src = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    dst = np.float32([[img_size[0] / 2 - dst_size, img_size[1] - bottom_offset],
                      [img_size[0] / 2 + dst_size, img_size[1] - bottom_offset],
                      [img_size[0] / 2 + dst_size, img_size[1] - 2 * dst_size - bottom_offset],
                      [img_size[0] / 2 - dst_size, img_size[1] - 2 * dst_size - bottom_offset]])
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, src, dst)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh_above(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    # check for sample rocks
    #rock_sample = find_rock_color(warped, rgb_thresh=(177, 148, 17))
    rock_sample = find_rock_color(warped, rgb_thresh=(160, 160, 75))
    # check for obstacles
    rock_obstacle = color_thresh_below(warped, rgb_thresh=(100, 100, 100))
    Rover.vision_image[:,:,0] = rock_obstacle
    Rover.vision_image[:,:,1] = rock_sample
    Rover.vision_image[:,:,2] = warped[:,:,0]
    # 5) Convert map image pixel values to rover-centric coords
    # Extract navigable terrain pixels
    xpix, ypix = rover_coords(threshed)
    # Extract sample posiion pixels
    xpix_sample, ypix_sample = rover_coords(rock_sample)
    # Extract obstacle posiion pixels
    xpix_obstacle, ypix_obstacle = rover_coords(rock_obstacle)
    # 6) Convert rover-centric pixel values to world coordinates
    navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, Rover.pos[0], 
                                Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale=10)
    # transform sample pixel values to world coordinates
    rock_x_world, rock_y_world = pix_to_world(xpix_sample, ypix_sample, Rover.pos[0], 
                                Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale=10)
    # transform obstacle pixel values to world coordinates
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obstacle, ypix_obstacle, Rover.pos[0], 
                                Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale=10)
    # if found samples, save
    #Rover.samples_pos[0] = rock_x_world
    #Rover.samples_pos[1] = rock_y_world
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 0] += 1
    # mark obstacle in the map
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    # mark sample in the map
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    # mark navigable in the map
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # 8) Convert rover-centric pixel positions to polar coordinates
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(xpix, ypix)
    # Update Rover pixel distances and angles
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles
        
    return Rover