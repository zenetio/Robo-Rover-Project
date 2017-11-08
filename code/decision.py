import numpy as np
import time
import math
#from PurePursuit import PurePursuit

def prepare_to_stop(Rover):
    # Set mode to "stop" and hit the brakes!
    Rover.throttle = 0
    # Set brake to stored brake value
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'stop'
    return Rover


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    # 'odo', ("%.2f" % Rover.odometry)
    #print('mode:', Rover.mode, 'dist:', np.mean(Rover.nav_dists), 'angle:', np.mean(Rover.nav_angles), 'near:', Rover.near_sample, 
    #'picking:', Rover.picking_up, 'send:', Rover.send_pickup, 'p_s:', Rover.picking_sample)
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # check if near sample enough to pick up
        if Rover.near_sample and Rover.vel > 0:
            # stop rover
            return prepare_to_stop(Rover)

        # Check if is picking up samples
        #if not Rover.picking_up:
        #    Rover = check_for_sample(Rover)
        if Rover.picking_up or Rover.picking_sample:
            return Rover    
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            vel = np.abs(Rover.vel)
            dt = time.time() - Rover.last_time
            Rover.last_time = time.time()      # save current time
            Rover.odometry += (vel * dt)        # update odometer
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < 0 or Rover.vel == 0:
                    prepare_to_turn(Rover)
                    return Rover
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                prepare_to_stop(Rover)

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop' and (not Rover.near_sample or not Rover.picking_up):
            #Rover.picking_up = 0
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
            # humm.. it looks like rover is blocked
            if Rover.time_stopped == 0:
                Rover.time_stopped = time.time()
            elif time.time() - Rover.time_stopped > 5:  # more than 5 sec?
                Rover.steer = 15
                Rover.throttle = Rover.throttle_set
                Rover.vel = -Rover.max_vel
                Rover.time_stopped = 0
                return Rover
            
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

def prepare_to_turn(Rover):
    Rover.mode = 'stop'
    Rover.throttle = 0
    Rover.steer = 0
    Rover.brake = Rover.brake_set