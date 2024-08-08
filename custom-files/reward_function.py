def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    
    # Reward for position on the track
    # Read input parameters
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering = abs(params['steering_angle']) # Only need the absolute steering angle

    # Calculate 3 marks that are farther and father away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.35 * track_width
    marker_4 = 0.5 * track_width

    # Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        reward = 4 
    elif distance_from_center <= marker_2:
        reward = 3
    elif distance_from_center <= marker_3:
        reward = 2
    elif distance_from_center <= marker_4:
        reward = 1 
    else:
        reward = 1e-3  # likely crashed/ close to off track

    # Steering penality threshold, change the number based on your action space setting
    ABS_STEERING_THRESHOLD = 13

    # Penalize reward if the car is steering too much
    if steering > ABS_STEERING_THRESHOLD:
        reward *= 0.8


    # reward for speed at different sections/turning on the track
    # read input parameter
    wp = params['closest_waypoints'][1]
    speed = params['speed']
    if reward >= 1:
        if (wp in (list(range(11,16)))) or (wp in (list(range(131, 136)))):
            if speed >= 2.5:
                reward += 1e-3 # high speed at turning
            else:
                reward += speed
        elif (wp in (list(range(41,53)))) or (wp in (list(range(65, 77)))) or (wp in (list(range(93, 105)))):
            if speed >= 2:
                reward += 1e-3 # high speed at turning
            else:
                reward += speed
        else:
            reward += speed
    

    return float(reward)
