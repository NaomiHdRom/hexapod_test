"""drive_my_robot controller."""

from controller import Robot

if __name__ == "__main__":

    # Create the Robot instance
    robot = Robot()

    # Time step
    timestep = int(robot.getBasicTimeStep())

    max_speed = 6.28

    # Motor instances
    left_motor = robot.getDevice('motor1')
    right_motor = robot.getDevice('motor2')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    num_side=4
    lenght_side=0.25
    wheel_radius=0.025
    linear_vel=wheel_radius * max_speed
    start_time=robot.getTime()
    
    angle_of_rotation=6.28/num_side
    distance_between_wheels = 0.090
    rate_of_rotation=(2*linear_vel)/distance_between_wheels
    
    duration_side=lenght_side/linear_vel

    duration_turn = angle_of_rotation/rate_of_rotation


    rot_start_time=start_time+duration_side
    rot_end_time=rot_start_time+duration_turn


    # Main loop
    while robot.step(timestep) != -1:
    
        current_time=robot.getTime()
        
        left_speed = 0.5 * max_speed
        right_speed = 0.5 * max_speed
        
        if rot_start_time < current_time < rot_end_time:
            left_speed = -max_speed
            right_speed = max_speed
            
        elif current_time>rot_end_time:
            rot_start_time = current_time+duration_side
            rot_end_time = rot_start_time+ duration_turn
            

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)