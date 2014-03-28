TIME_STEP = 128;
SENSOR_COUNT = 8;
WHEEL_RADIUS = 8;
ROBOT_DIAMETER = 26.5;

LT = 2; % top-left sensor
RT = 5; % top-right sensor
R = 6; % rightmost sensor
L = 1; % leftmost sensor

% Parameters used for obstacle detection
WALL_DISTANCE = 680;
CORNER_MAX = 720;
WALL_MIN = 700;
WALL_MAX = 650;

% Internal position tracking variables
x_dist = 0;
y_dist = 0;
theta = 0;

% State variables
turn_mode = false;
should_run = true;
away_from_beginning = false;
wall_found = false;

% These 2 sensors should have a reference value if we are to follow
% a wall at constant speed.
reference = -ones(1, SENSOR_COUNT);
reference(RT) = 380;
reference(R) = WALL_DISTANCE;

% Base and maximum speeds
LEFT_SPEED = 3;
RIGHT_SPEED = 3;
MAX_SPEED = 10;

% Enabling the motor rotation counts:
wb_differential_wheels_enable_encoders(TIME_STEP);
wb_differential_wheels_set_encoders(0, 0);

% Set up an array to keep a history of errors in order to compute
% the integral adjustment.
error_history = zeros(1, SENSOR_COUNT);

% Get and enable distance sensors
for i=1:SENSOR_COUNT
    ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
    wb_distance_sensor_enable(ps(i),TIME_STEP);
end

% Main loop
while (wb_robot_step(TIME_STEP) ~= -1) & should_run

    % Read all distance sensors and calculate errors with respect to
    % reference values for the relevant ones.
    error_values = zeros(1, SENSOR_COUNT);
    for i=1:SENSOR_COUNT
        sensor_values(i) = wb_distance_sensor_get_value(ps(i));
        if reference(i) ~= -1
            error_values(i) = sensor_values(i) - reference(i);
        end
    end
    % disp(sensor_values(3:6));
    % If we haven't reached any wall at all:
    if ~any(sensor_values(3:6) > WALL_DISTANCE) && ~wall_found
        % While there is no wall in front or to the right, simply go straight.
        right_speed = RIGHT_SPEED;
        left_speed = LEFT_SPEED;
    % If we are near a wall:
    else
        wall_found = true;
        % If we need to turn:
        if turn_mode || (any(sensor_values(3:4) > WALL_DISTANCE-40)) || ...
           (sensor_values(RT) > CORNER_MAX) || (sensor_values(R) > WALL_MIN)
            % Avoid walls and other obstacles in front by stopping and turning until
            % the way ahead is clear.
            turn_mode = true;
            right_speed = -RIGHT_SPEED;
            left_speed = LEFT_SPEED;
            if ~any(sensor_values(3:4))
                if sensor_values(RT) < CORNER_MAX
                    if sensor_values(R) < WALL_MIN
                        turn_mode = false;
                    end
                elseif sensor_values(R) < WALL_MAX
                    turn_mode = false;
                end
            end
        % Else we just follow the wall:
        else
            % Compute PID adjustments and add them to the default speeds of the motors.
            % P component
            [add_right, add_left] = p_component(error_values);
            right_speed = RIGHT_SPEED + add_right;
            left_speed = LEFT_SPEED + add_left;
            % I component
            [add_right, add_left, error_history] = ...
                            i_component(error_values, error_history);
            right_speed = right_speed + add_right;
            left_speed = left_speed + add_left;
        end
        % We get the distance traveled by left and right wheel in millimeters:
        left_motor_dist = WHEEL_RADIUS * wb_differential_wheels_get_left_encoder()/100;
        right_motor_dist = WHEEL_RADIUS * wb_differential_wheels_get_right_encoder()/100;
        % Update the coordinate and angle representation:
        x_dist = x_dist + 0.5 * (left_motor_dist + right_motor_dist) * cos(theta);
        y_dist = y_dist + 0.5 * (left_motor_dist + right_motor_dist) * sin(theta);
        theta = theta - 0.5 * (left_motor_dist - right_motor_dist)/(ROBOT_DIAMETER);
        % We set away_from_beginning to true once we get far from the origin
        % so that we could know when the robot has completed the lap and needs to stop.
        if sqrt(x_dist^2 + y_dist^2) > 30
            away_from_beginning = true;
        end
        disp([x_dist y_dist theta]);
        % If we are very close to the origin and we're completing the lap:
        if sqrt(x_dist^2 + y_dist^2) < 20 & away_from_beginning
            should_run = false;
            right_speed = 0;
            left_speed = 0;
        end
    end
    % Cap speeds in order to avoid erratic movements.
    right_speed = max(-MAX_SPEED, right_speed);
    right_speed = min(MAX_SPEED, right_speed);
    left_speed = max(-MAX_SPEED, left_speed);
    left_speed = min(MAX_SPEED, left_speed);
    % We set the speed and reset the motor rotation counts:
    wb_differential_wheels_set_speed(right_speed, left_speed);
    wb_differential_wheels_set_encoders(0, 0);
end