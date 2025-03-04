clear;
clc;
syms q1 q2 q3 q4 q5 q6
syms dq1 dq2 dq3 dq4 dq5 dq6
syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6

q = [q1 q2 q3 q4 q5 q6];
dq = [dq1 dq2 dq3 dq4 dq5 dq6];
ddq = [ddq1 ddq2 ddq3 ddq4 ddq5 ddq6];

q0 = [0 0 0 0 0 0]; % Initial robot position
qf = [pi/4 pi/2 0 -pi/2 0 0]; % Final robot position

% qf = [pi/2 pi/6 0 0 -pi/6 0];
% Generate time values and the trajectory for each joint.
t = linspace(0,3,50);
p1 = quinticTrajectory(q0(1),0,0,qf(1),0,0,t);
p2 = quinticTrajectory(q0(2),0,0,qf(2),0,0,t);
p3 = quinticTrajectory(q0(3),0,0,qf(3),0,0,t);
p4 = quinticTrajectory(q0(4),0,0,qf(4),0,0,t);
p5 = quinticTrajectory(q0(5),0,0,qf(5),0,0,t);
p6 = quinticTrajectory(q0(6),0,0,qf(6),0,0,t);

% Combine the paths into a cell array
paths = {p1, p2, p3, p4, p5, p6};

% create the dynamic model
dyn_model = dynamicModel(10);

% Calculate the torque
tau = calculateTorque(paths, q, dq, ddq, dyn_model);

% Plot the torque required at each joint.
plot_joints(t, tau);

% Plot the stick plot of the robot arm's movement.
stick_plot(paths, t);

function plot_joints(t, Tau)
figure;
hold on;
plot(t, Tau(1, :), 'r', 'DisplayName', 'Joint #1');
plot(t, Tau(2, :), 'g', 'DisplayName', 'Joint #2');
plot(t, Tau(3, :), 'b', 'DisplayName', 'Joint #3');
xlabel('Time (s)');
ylabel('Torque (N*m)');
title('Required Torque input for the Robot Arm (Joints 1-3)');
legend show;
grid on;
hold off;

figure;
hold on;
plot(t, Tau(4, :), 'k--', 'DisplayName', 'Joint #4');
plot(t, Tau(5, :), 'm', 'DisplayName', 'Joint #5');
plot(t, Tau(6, :), 'b--', 'DisplayName', 'Joint #6');
xlabel('Time (s)');
ylabel('Torque (N*m)');
title('Required Torque input for the Robot Arm (Joints 4-6)');
legend show;
grid on;
hold off;
end

function stick_plot(paths, t)
    % Define the robot's DH parameters
    d = [330 0 0 335 0 80];
    a = [50 330 35 0 0 0];
    alpha = [pi/2 0 pi/2 -pi/2 pi/2 0];
    
    % Forward Kinematics for initial and final positions
    q0 = cellfun(@(path) path(1, 1), paths);
    qf = cellfun(@(path) path(1, end), paths);
    [T01_init, T02_init, T03_init, T04_init, T05_init, TF_init] = FK(q0, d, a, alpha);
    [T01_final, T02_final, T03_final, T04_final, T05_final, TF_final] = FK(qf, d, a, alpha);
    
    % Extract positions
    positions_init = [0 0 0; T01_init(1:3, 4).'; T02_init(1:3, 4).'; T03_init(1:3, 4).'; T04_init(1:3, 4).'; T05_init(1:3, 4).'; TF_init(1:3, 4).'];
    positions_final = [0 0 0; T01_final(1:3, 4).'; T02_final(1:3, 4).'; T03_final(1:3, 4).'; T04_final(1:3, 4).'; T05_final(1:3, 4).'; TF_final(1:3, 4).'];
    
    % Plot the initial and final positions
    figure;
    plot3(positions_init(:,1), positions_init(:,2), positions_init(:,3), 'bo-', 'LineWidth', 2, 'DisplayName', 'Initial Position');
    hold on;
    plot3(positions_final(:,1), positions_final(:,2), positions_final(:,3), 'ro-', 'LineWidth', 2, 'DisplayName', 'Final Position');
    
    % Plot the movement paths for each joint
    num_points = length(t);
    for i = 2:length(positions_init)
        joint_path = zeros(3, num_points);
        for j = 1:num_points
            q_t = cellfun(@(path) path(1, j), paths);
            [T01, T02, T03, T04, T05, TF] = FK(q_t, d, a, alpha);
            T_all = {T01, T02, T03, T04, T05, TF};
            joint_path(:, j) = T_all{i-1}(1:3, 4);
        end
        plot3(joint_path(1,:), joint_path(2,:), joint_path(3,:), 'k--', 'LineWidth', 1);
    end
    
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    title('Stick Plot of Robot Arm Movement');
    legend show;
    grid on;
    hold off;
end


