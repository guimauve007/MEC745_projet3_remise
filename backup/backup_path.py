def follow_path(path):
    """ Follow the path using proportional control. """
    
    if not path or len(path) < 2:
        print("Path is too short or empty!")
        return

    print("Starting path following...")
    waypoint_index = 0
    while waypoint_index < len(path) - 1:
        current_x, current_y, current_theta = get_robot_map_pixel_position()

        target_x, target_y = path[waypoint_index].x, path[waypoint_index].y
        target_x -= origin_x
        target_y = -(target_y - origin_y)


        distance = compute_distance(current_x, current_y, target_x, target_y)
        heading_error = compute_heading_error(current_x, current_y, current_theta, target_x, target_y)
        print(f"waypoint position:{target_x}, {target_y}, {heading_error}")
        
        # Proportional control for angular velocity
        angular_velocity = max(Kp_angular * heading_error, MAX_ANGULAR_SPEED)
        
        # linear_velocity = min(Kp_linear * distance, max_linear_speed)
        if heading_error < 0.25:
            linear_velocity = max(Kp_linear * distance, MAX_LINEAR_SPEED)
            
        else:
            linear_velocity = 0.01
            
        # Move the robot
        move_robot(linear_velocity, angular_velocity)

        # Check if the waypoint is reached
        if distance < 1:  # Tolerance for reaching waypoint
            waypoint_index += 1
            print(f"Reached waypoint {waypoint_index}, moving to next...")

        rospy.sleep(0.1)

    print("Path completed.")
    move_robot(0, 0)  # Stop the robot