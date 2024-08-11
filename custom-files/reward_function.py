def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    
    # Reward for position on the track
    # Read input parameters
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering = abs(params['steering_angle']) # Only need the absolute steering angle
    wp = params['closest_waypoints'][1]
    speed = params['speed']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']

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

    if not all_wheels_on_track:
        reward *= 0.1

    # Steering penality threshold, change the number based on your action space setting
    ABS_STEERING_THRESHOLD = 15

    # Penalize reward if the car is steering too much
    if steering > ABS_STEERING_THRESHOLD:
        reward *= 0.7


    # reward for speed at different sections/turning on the track
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
    
    # Reward higher speeds more significantly
    SPEED_THRESHOLD = 2.5
    if speed > SPEED_THRESHOLD:
        reward *= 1.5

    # Additional reward for completing the lap faster
    if progress == 100:
        reward += 10.0  # Large reward for completing the lap

    return float(reward)
