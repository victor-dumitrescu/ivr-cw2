function [add_right, add_left, new_error_history] = ...
         i_component (error_values, error_history)
    % Integral component of the PID controller.
    I_GAIN = [0 0 0 0 0.0001 0.0001 0 0];
    epsilon = 10 * ones(1,8);

    if all(abs(error_values) < epsilon)
        % Reset error history
        new_error_history = zeros(1, 8);
        add_right = 0;
        add_left = 0;
    else
        % Update error history
        new_error_history = error_history + I_GAIN .* error_values;
        add_left = 0;
        add_right = 0;

        for i=1:8
            if new_error_history(i) > 0
                add_left = add_left + new_error_history(i);
            else
                add_right = add_right - new_error_history(i);
            end
    end
end