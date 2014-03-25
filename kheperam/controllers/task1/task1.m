TIME_STEP = 64;
N = 8;

LT = 2; % top-left sensor
RT = 5; % top-right sensor
R = 6; % rightmost sensor
L = 1; % leftmost sensor

wall_distance = 700;
corner_max = 800;
wall_min = 800;
wall_max = 900;

x_dist = 0;
y_dist = 0;
theta = 0;

turn_mode = false;
obstacle_mode = false;
should_run = true;
away_from_beginning = false;

% These 2 sensors should have a reference value if we are to follow
% a wall at constant speed.
reference = -ones(1, N);
reference(RT) = 400;
reference(R) = wall_distance;

LEFT = 3;
RIGHT = 3;
MAX_SPEED = 10;

% Set up an array to keep a history of errors in order to compute
% the integral adjustment.
error_history = zeros(1, N);

% get and enable distance sensors
for i=1:N
    ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
    wb_distance_sensor_enable(ps(i),TIME_STEP);
end

% Calling MATLAB desktop version
% desktop;

% Initially set the robot in motion.
wb_differential_wheels_set_speed(RIGHT, LEFT);

% Main loop
while wb_robot_step(TIME_STEP) ~= -1 && should_run
    % Read all distance sensors and calculate errors with respect to
    % reference values for the relevant ones.
    error_values = zeros(1, N);
    for i=1:N
        sensor_values(i) = wb_distance_sensor_get_value(ps(i));
        if reference(i) ~= -1 
            error_values(i) = sensor_values(i) - reference(i);
        end
    end
    % Avoid walls and other obstacles in front by stopping and turning until
    % the way ahead is clear. This should also preserve the lateral distance
    % to the walls.
    if turn_mode || (any(sensor_values(3:4) > wall_distance)) || ...
       (sensor_values(RT) > corner_max) || (sensor_values(LT) > corner_max) || ...
       (sensor_values(R) > wall_min) || (sensor_values(L) > wall_min)
        
        turn_mode = true;

        right_speed = -RIGHT;
        left_speed = LEFT;

        if ~any(sensor_values(3:4)) || ~sensor_values(RT) && ~sensor_values(LT) || ...
            (sensor_values(R) < wall_max) || (sensor_values(L) < wall_max)
        
            turn_mode = false;
        end

    else
        % Compute PID adjustments and add them to the default speeds of the motors.
        % P component
        [add_right, add_left] = p_component(error_values);
        right_speed = RIGHT + add_right;
        left_speed = LEFT + add_left;

        % I component
        [add_right, add_left, error_history] = ...
                        i_component(error_values, error_history);

        right_speed = right_speed + add_right;
        left_speed = left_speed + add_left;
    end

    % Cap speeds in order to avoid erratic movements.
    right_speed = max(-MAX_SPEED, right_speed);
    right_speed = min(MAX_SPEED, right_speed);
    left_speed = max(-MAX_SPEED, left_speed);
    left_speed = min(MAX_SPEED, left_speed);

    %disp([right_speed, left_speed]);
    wb_differential_wheels_set_speed(right_speed, left_speed);
    x_dist = x_dist + 0.5 * (left_speed + right_speed) * cos(theta);
    y_dist = y_dist + 0.5 * (left_speed + right_speed) * sin(theta);
    theta = theta - 0.5 * (left_speed - right_speed)/(2 * 6*4.3);
    %disp(theta);
    if sqrt(x_dist^2 + y_dist^2) > 500
        away_from_beginning = true;
    end
    if sqrt(x_dist^2 + y_dist^2) < 100 && away_from_beginning
        should_run = false;
        disp('Done!');
    end
end
