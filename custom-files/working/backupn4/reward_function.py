import math
def reward_function(params):
  
    all_wheels_on_track = params['all_wheels_on_track']
    steps = params['steps']
    progress = params['progress']
    speed = params['speed']
    steering_angle = params['steering_angle']
    offtrack = params['is_offtrack']
    distance_from_center = params['distance_from_center']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    next_point = waypoints[closest_waypoints[1]]
    track_width = params['track_width']
    left_of_center = params['is_left_of_center']
    reward = 1
    slow_speed_threshold = 1.4
    fast_speed_threshold = 3.2
    step_goal = 250
    
    #based on the track points
    #Fast
    if next_point in range(0,10) or next_point in range(19,31) or next_point in range(139,153) or next_point in range(108,130):
        if all_wheels_on_track and left_of_center:
            reward += 10   
        else:
            reward -= 1

        if speed >= fast_speed_threshold:
            reward += 10
        else:
            reward -= 1
    else:
        if next_point in range(68,79) and not left_of_center and all_wheels_on_track:
        #curve
            reward += 11
        elif left_of_center and all_wheels_on_track:
            reward += 10
        else:
            reward -= 1
            
        if speed >= slow_speed_threshold and all_wheels_on_track:
            reward += 12
        else:
            reward += 1e-3
           
    #Add for model 14           
    if (steps % 100) == 0 and progress > (steps / step_goal) * 100 :
        reward += 10.0
    ABS_STEERING_THRESHOLD = 20 

    # Penalize reward if the car is steering too much
    steering = abs(params['steering_angle'])
    if steering > ABS_STEERING_THRESHOLD:
        reward -= 1
	
                   
    benchmark_time = 20.0	
    #Get reward if completes the lap and more reward if it is faster than benchmark_time    
    if progress == 100:
        if round(steps/15,1)<benchmark_time:
            reward+=100*round(steps/15,1)/benchmark_time
        else:
            reward += 100
    elif offtrack:
        reward-=50  
    return float(reward)

