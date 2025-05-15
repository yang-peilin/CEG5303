%% Quadcopter simulation - Two UAVs Example
% Each UAV uses force in z and torques in roll/pitch/yaw as control input.
% This script just generates references (two line trajectories) and runs two simulations.
% The results are then compared and displayed.

close all;
clear;
clc;

%% Basic parameters
g = 9.81;
m = 0.468; % mass of the quadcopter

% PID gains - for demonstration, we keep them the same for both UAVs
Kp_z = 4;
Ki_z = 0.1;
Kd_z = 3;

Kp_pos = 6;
Ki_pos = 0.1;
Kd_pos = 4;

Kp_att = 8;
Ki_att = 0.1;
Kd_att = 1;

Kp_psi = 8;
Ki_psi = 0.1;
Kd_psi = 1;

%% Simulation settings
t = 120;              % total simulation time
dt = 0.01;            % time step
t_ref = 0:dt:t;       % time vector

%% ========== UAV1: Generate simple line trajectory ==========
waypoints_uav1 = [
    0   0   1;
    0   4   1;
    4   4   1;
    4   0   1;
    0   0   1;
    -4  0   1;
    -4  -4  1;
    0   -4  1;
    0   0   1;
];

% Extract x,y,z
x1 = waypoints_uav1(:,1);
y1 = waypoints_uav1(:,2);
z1 = waypoints_uav1(:,3);

% Repeat waypoints to fill entire time array
duration1 = floor(length(t_ref) / size(waypoints_uav1,1));
x1_ref_array = repelem(x1, duration1);
y1_ref_array = repelem(y1, duration1);
z1_ref_array = repelem(z1, duration1);

% Make sure the reference vectors match length(t_ref)
if length(x1_ref_array) < length(t_ref)
    x1_ref_array(length(t_ref)) = x1_ref_array(end);
    y1_ref_array(length(t_ref)) = y1_ref_array(end);
    z1_ref_array(length(t_ref)) = z1_ref_array(end);
end

% Combine time & reference for UAV1
x_ref1 = [t_ref' , x1_ref_array];
y_ref1 = [t_ref' , y1_ref_array];
z_ref1 = [t_ref' , z1_ref_array];

%% ========== UAV2: Generate another line trajectory ==========
% E.g., from (0,2,1) --> (5,2,1)
waypoints_uav2 = [
    0   0   2;
    0   4   2;
    -4  4  2;
    -4  0   2;
    0   0   2;
    4   0   2;
    4   -4  2;
    0   -4  2;
    0   0   2;
];

x2 = waypoints_uav2(:,1);
y2 = waypoints_uav2(:,2);
z2 = waypoints_uav2(:,3);

duration2 = floor(length(t_ref) / size(waypoints_uav2,1));
x2_ref_array = repelem(x2, duration2);
y2_ref_array = repelem(y2, duration2);
z2_ref_array = repelem(z2, duration2);

if length(x2_ref_array) < length(t_ref)
    x2_ref_array(length(t_ref)) = x2_ref_array(end);
    y2_ref_array(length(t_ref)) = y2_ref_array(end);
    z2_ref_array(length(t_ref)) = z2_ref_array(end);
end

x_ref2 = [t_ref' , x2_ref_array];
y_ref2 = [t_ref' , y2_ref_array];
z_ref2 = [t_ref' , z2_ref_array];


out = sim('Quadcopter_simulation_xry', t);

%% ========== 提取第一架无人机的状态 ==========
time1   = out.states1.Time;      % 时间向量
x_out1  = out.states1.Data(:,1);
y_out1  = out.states1.Data(:,2);
z_out1  = out.states1.Data(:,3);
% 若有速度信息，可以是(:,4:6)
phi_out1   = out.states1.Data(:,7);
theta_out1 = out.states1.Data(:,8);
psi_out1   = out.states1.Data(:,9);

%% ========== 提取第二架无人机的状态 ==========
time2   = out.states2.Time;
x_out2  = out.states2.Data(:,1);
y_out2  = out.states2.Data(:,2);
z_out2  = out.states2.Data(:,3);
phi_out2   = out.states2.Data(:,7);
theta_out2 = out.states2.Data(:,8);
psi_out2   = out.states2.Data(:,9);

%% ========== 提取两架无人机的控制输入(推力+力矩) ==========
u_T1     = out.control1.Data(:,1);
u_phi1   = out.control1.Data(:,2);
u_theta1 = out.control1.Data(:,3);
u_psi1   = out.control1.Data(:,4);

u_T2     = out.control2.Data(:,1);
u_phi2   = out.control2.Data(:,2);
u_theta2 = out.control2.Data(:,3);
u_psi2   = out.control2.Data(:,4);

%% ========== 1) Plot X/Y/Z vs Time, 对比真实输出与参考 ==========
figure('Name','XYZ vs Time','NumberTitle','off');

% -- X(t)
subplot(3,1,1); hold on; grid on;
plot(time1, x_out1, '-b');                    % UAV1 实际
plot(x_ref1(:,1), x_ref1(:,2), '--b');        % UAV1 参考 (时间列, X列)
plot(time2, x_out2, '-r');                    % UAV2 实际
plot(x_ref2(:,1), x_ref2(:,2), '--r');        % UAV2 参考
title('X to time'); ylabel('X');
legend({'UAV1 State','UAV1 Ref','UAV2 State','UAV2 Ref'},'Location','best');

% -- Y(t)
subplot(3,1,2); hold on; grid on;
plot(time1, y_out1, '-b');
plot(y_ref1(:,1), y_ref1(:,2), '--b');
plot(time2, y_out2, '-r');
plot(y_ref2(:,1), y_ref2(:,2), '--r');
title('Y to time'); ylabel('Y');
legend({'UAV1 State','UAV1 Ref','UAV2 State','UAV2 Ref'},'Location','best');

% -- Z(t)
subplot(3,1,3); hold on; grid on;
plot(time1, z_out1, '-b');
plot(z_ref1(:,1), z_ref1(:,2), '--b');
plot(time2, z_out2, '-r');
plot(z_ref2(:,1), z_ref2(:,2), '--r');
title('Z to time'); ylabel('Z');
xlabel('Time (s)');
legend({'UAV1 State','UAV1 Ref','UAV2 State','UAV2 Ref'},'Location','best');

%% ========== 2) Plot Roll/Pitch/Yaw vs Time (无参考，只显示实际姿态) ==========
figure('Name','phi/theta/psi vs Time','NumberTitle','off');

% -- phi(t)
subplot(3,1,1); hold on; grid on;
plot(time1, phi_out1, '-b');
plot(time2, phi_out2, '-r');
title('phi to time'); ylabel('phi');
legend({'UAV1','UAV2'},'Location','best');

% -- theta(t)
subplot(3,1,2); hold on; grid on;
plot(time1, theta_out1, '-b');
plot(time2, theta_out2, '-r');
title('theta to time'); ylabel('theta');
legend({'UAV1','UAV2'},'Location','best');

% -- psi(t)
subplot(3,1,3); hold on; grid on;
plot(time1, psi_out1, '-b');
plot(time2, psi_out2, '-r');
title('psi to time'); ylabel('psi');
xlabel('Time (s)');
legend({'UAV1','UAV2'},'Location','best');

%% ========== 3) 3D Trajectory + Waypoints (如同单无人机版本) ==========
figure('Name','3D Trajectory','NumberTitle','off');
plot3(x_out1, y_out1, z_out1, '-b','LineWidth',1.5); hold on;
plot3(x_out2, y_out2, z_out2, '-r','LineWidth',1.5);

% 航点 (可选)
plot3(x1, y1, z1, 'ob','MarkerFaceColor','b');
plot3(x2, y2, z2, 'or','MarkerFaceColor','r');

legend({'UAV1','UAV2','Waypoints1','Waypoints2'},'Location','best');
box on; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Quadcopter 3D Position');

%% ========== 4) Plot Control usage (T, torque phi/theta/psi) ==========
figure('Name','Control usage','NumberTitle','off');

% -- Thrust
subplot(4,1,1); hold on; grid on;
plot(time1, u_T1, '-b');
plot(time2, u_T2, '-r');
title('Control usage - Thrust'); ylabel('T');
legend({'UAV1','UAV2'},'Location','best');

% -- torque phi
subplot(4,1,2); hold on; grid on;
plot(time1, u_phi1, '-b');
plot(time2, u_phi2, '-r');
title('Torque in Roll'); ylabel('\tau_\phi');

% -- torque theta
subplot(4,1,3); hold on; grid on;
plot(time1, u_theta1, '-b');
plot(time2, u_theta2, '-r');
title('Torque in Pitch'); ylabel('\tau_\theta');

% -- torque psi
subplot(4,1,4); hold on; grid on;
plot(time1, u_psi1, '-b');
plot(time2, u_psi2, '-r');
title('Torque in Yaw'); ylabel('\tau_\psi');
xlabel('Time (s)');
legend({'UAV1','UAV2'},'Location','best');
