% Set the save path
savePath = 'D:\Thesis\Testing_and_results\CarMaker_results\PID_results';

% Figure 1: Car Trajectories at Different Speeds
figure;
hold on;
plot(Path_x40, Path_y40, 'K.-', 'LineWidth', 2, 'DisplayName', 'Reference path');
plot(CarX15, CarY15, 'r', 'MarkerSize', 10, 'DisplayName', 'Car trajectory (20 m/s)');
plot(CarX40, CarY40, 'g', 'MarkerSize', 10, 'DisplayName', 'Car trajectory (10 m/s)');
plot(CarX, CarY, 'b', 'MarkerSize', 10, 'DisplayName', 'Car trajectory (30 m/s)');
xlabel('x [m]', 'FontSize', 14);
ylabel('y [m]', 'FontSize', 14);
title('Path Tracking - Car Trajectories at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;
saveas(gcf, fullfile(savePath, 'Car_Trajectories_Speed_Variation.png')); % Save the figure as a PNG

% Figure 2: Brake Commands
figure;
hold on;
plot(Time,Brake_command, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,Brake_command15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,Brake_command40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Brake Command', 'FontSize', 14);
title('Brake Commands at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;
saveas(gcf, fullfile(savePath, 'Brake_Commands_Speed_Variation.png')); % Save the figure as a PNG

% ... Repeat similar blocks for other variables ...

% Figure 3: Yaw Angle and Yaw Rate
figure;
subplot(2, 1, 1);
hold on;
plot(Time, CarYaw, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15, CarYaw15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,CarYaw40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Yaw Angle [rad]', 'FontSize', 14);
title('Yaw Angle at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;

subplot(2, 1, 2);
hold on;
plot(Time,CarYaw_rate, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15, CarYaw_rate15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,CarYaw_rate40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Yaw Rate [rad/s]', 'FontSize', 14);
title('Yaw Rate at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;

saveas(gcf, fullfile(savePath, 'Yaw_Angle_Rate_Speed_Variation.png')); % Save the figure as a PNG

% Figure 4: Slip Angle
figure;
hold on;
plot(Time,Slip_angle, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,Slip_angle15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,Slip_angle40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Slip Angle [rad]', 'FontSize', 14);
title('Slip Angle at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;
saveas(gcf, fullfile(savePath, 'Slip_Angle_Speed_Variation.png')); % Save the figure as a PNG

% ... Repeat similar blocks for other variables ...

% Figure 5: Wheel Torques
figure;
subplot(2, 2, 1);
hold on;
plot(Time,FLTrq_Brake, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,FLTrq_Brake15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,FLTrq_Brake40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Front Left Wheel Torque [N.m]', 'FontSize', 14);
title('Front Left Wheel Torque at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;

subplot(2, 2, 2);
hold on;
plot(Time,FRTrq_Brake, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,FRTrq_Brake15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,FRTrq_Brake40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Front Right Wheel Torque [N.m]', 'FontSize', 14);
title('Front Right Wheel Torque at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;

subplot(2, 2, 3);
hold on;
plot(Time,RLTrq_Brake, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,RLTrq_Brake15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,RLTrq_Brake40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Rear Left Wheel Torque [N.m]', 'FontSize', 14);
title('Rear Left Wheel Torque at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;

subplot(2, 2, 4);
hold on;
plot(Time,RRTrq_Brake, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,RRTrq_Brake15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,RRTrq_Brake40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Rear Right Wheel Torque [N.m]', 'FontSize', 14);
title('Rear Right Wheel Torque at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;

saveas(gcf, fullfile(savePath, 'Wheel_Torques_Speed_Variation.png')); % Save the figure as a PNG


% Figure 7: Slip Angles
figure;
hold on;
plot(Time,Slip_angle, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15, Slip_angle15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,Slip_angle40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Slip Angle [rad]', 'FontSize', 14);
title('Slip Angle at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;

% Figure 8: Speed
figure;
hold on;
plot(Time,Speed, 'r-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,Speed15, 'b-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,Speed40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Speed [m/s]', 'FontSize', 14);
title('Vehicle Speed at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;
title('Speed controller', 'FontSize', 16);
% ... Repeat similar blocks for other variables ...

% Save the figure as a PNG
saveas(gcf, fullfile(savePath, 'BrakeCommands_SlipAngles_Speed.png'));
% Figure 9: Steering
figure;
hold on;
plot(Time,Steering, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,Steering15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,Steering40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Steering [rad]', 'FontSize', 14);
title('Steering at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;
saveas(gcf, fullfile(savePath, 'Steering_Angle.png'));
% Figure 10: Throttle
figure;
hold on;
plot(Time,Throttle, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,Throttle15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,Throttle40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Throttle [N]', 'FontSize', 14);
title('Throttle at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;
saveas(gcf, fullfile(savePath, 'Throttle.png'));
% Figure 11: Trajectory Errors
figure;
hold on;
plot(Time,Traj_error, 'b-', 'LineWidth', 2, 'DisplayName', '30 m/s speed');
plot(Time15,Traj_error15, 'r-', 'LineWidth', 2, 'DisplayName', '20 m/s speed');
plot(Time40,Traj_error40, 'g-', 'LineWidth', 2, 'DisplayName', '10 m/s speed');
xlabel('Time [s]', 'FontSize', 14);
ylabel('Trajectory Error [m]', 'FontSize', 14);
title('Trajectory Error at Different Speeds', 'FontSize', 16);
legend('Location', 'Best', 'FontSize', 12);
hold off;

% Save the figure as a PNG
saveas(gcf, fullfile(savePath, 'TrajectoryErrors.png'));

