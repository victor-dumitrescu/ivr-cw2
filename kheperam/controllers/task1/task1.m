TIME_STEP = 64;
N = 8;

% get and enable distance sensors
for i=1:N
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i),TIME_STEP);
end

% Calling MATLAB desktop version
% wb_differential_wheels_set_speed(1, -1);
wb_differential_wheels_set_speed(10, -10);
desktop;

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the controll to the keyboard
while wb_robot_step(TIME_STEP) ~= -1

  % read all distance sensors
       for i=1:N
           sensor_values(i) = wb_distance_sensor_get_value(ps(i));
       end
  % display all distance sensors values
       sensor_values

  left_speed=1;
  right_speed=1;
  %set_speeds(left_speed,right_speed);
  if sensor_values(1)+sensor_values(2)+sensor_values(3)>10
    wb_differential_wheels_set_speed(1, -1);
  elseif sensor_values(4)+sensor_values(5)+sensor_values(6)>10
    wb_differential_wheels_set_speed(-10, 10);
  else
     wb_differential_wheels_set_speed(10, 10);
  end;
  %control goes to the keyboard
  %keyboard;
end
