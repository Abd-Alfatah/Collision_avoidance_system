speed = 0:0.1:100;
steering_angle =-3.14/4:0.00628/4:3.14/4;  % Matches size of speed array
brake_select = ones(length(speed), length(steering_angle));
v_min = ones(1,length(speed));
curvature = 0.000:0.001:1;
% Vehicle parameters (adjust as needed)
L = 2.975;  % Wheelbase
W = 1.8;  % Track width
mu = 1; % Coefficient of friction (conservative estimate)
g = 9.81; % Acceleration due to gravity
R = ones(1,length(speed));
for i = 1:length(speed)
    for j = 1:length(steering_angle)
        % Calculate turning radius using steering angle
        R(j) = L*tan(pi/2-steering_angle(j));

        % Calculate minimum safe speed
        v_min(j) = sqrt(mu * g * abs(R(j)));

        if speed(i) > v_min(j)
            brake_select(i, j) = speed(i) / v_min(j);  % Apply braking based on ratio
        else
            brake_select(i, j) = 1;  % No braking needed
        end
    end
end
