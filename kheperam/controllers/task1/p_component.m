function [add_right, add_left] = p_component (error_values, reference)
    % Proportional component of the PID controller.
    P_GAIN = [0 0 0 0 0.05 0.05 0 0];

    add_left = 0;
    add_right = 0;

    % Negative error: getting too far
    % Positive error: getting too close
    for i=1:8
        if error_values(i) > 0
            add_left = add_left + P_GAIN(i) * error_values(i);
        else
            add_right = add_right - P_GAIN(i) * error_values(i);
        end

end