TIME_STEP = 64;
N = 8;

RT = 5; % top-right sensor
R = 6; % rightmost sensor
wall_distance = 700; 

TURN_MODE = false;

% These 2 sensors should have a reference value if we are to follow
% a wall at constant speed.
reference = -ones(1, 8);
reference(RT) = 400;
reference(R) = wall_distance;

LEFT = 1;
RIGHT = 1;

% get and enable distance sensors
for i=1:N
    ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
    wb_distance_sensor_enable(ps(i),TIME_STEP);
end

% Calling MATLAB desktop version
% desktop;

% Initially set the robot in motion.
wb_differential_wheels_set_speed(1, 1);

% Main loop
while wb_robot_step(TIME_STEP) ~= -1

    % Read all distance sensors and calculate errors with respect to
    % reference values for the relevant ones.
    error_values = zeros(1, 8);
    for i=1:N
        sensor_values(i) = wb_distance_sensor_get_value(ps(i));
        if reference(i) ~= -1 
            error_values(i) = sensor_values(i) - reference(i);
        end
    end
    % error_values

    % Avoid walls and other obstacles in front by stopping and turning until
    % the way ahead is clear. This should also preserve the lateral distance
    % to the walls.
    if TURN_MODE || (min(sensor_values(3:4)) > wall_distance)
        % Turn mode will be active until front sensors no longer detect an obstacle.
        TURN_MODE = true;
        right_speed = -1;
        left_speed = 1;
        if ~any(sensor_values(3:4))
            TURN_MODE = false;
        end
    else
        % Compute PID adjustments and add them to the default speeds of the motors.
        [add_right, add_left] = p_component(error_values, reference);
        right_speed = RIGHT + add_right;
        left_speed = LEFT + add_left;
    end

    % Cap speeds in order to avoid erratic movements.
    right_speed = max(-5, right_speed);
    right_speed = min(5, right_speed);
    left_speed = max(-5, left_speed);
    left_speed = min(5, left_speed);

    disp([right_speed, left_speed]);
    wb_differential_wheels_set_speed(right_speed, left_speed);

    %control goes to the keyboard
    %keyboard;
end
