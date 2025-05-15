%Quadcopter simulation and PID control based on cartesian coordinate system. 
%Input to the system is force in z, torques in roll pitch and yaw
close all;
clear;
clc;
g = 9.81;
m = 0.468; %mass of quadcopter

%TODO_5: Set PID gains.
% 设定 PID 增益系数

% 高度控制 (Kp_z, Ki_z, Kd_z)
Kp_z = 4;
Ki_z = 0.1;
Kd_z = 3;

% 位置控制 (Kp_pos, Ki_pos, Kd_pos)
Kp_pos = 6;
Ki_pos = 0.1;
Kd_pos = 4;

% 姿态控制 (Kp_att, Ki_att, Kd_att)
Kp_att = 8;
Ki_att = 0.1;
Kd_att = 1;

% 偏航角控制 (Kp_psi, Ki_psi, Kd_psi)
Kp_psi = 8;
Ki_psi = 0.1;
Kd_psi = 1;


%Generate some waypoints for the quadcopter to follow (every row is x,y,z)
%Free free to generate your own trajectory by modifying the waypoints.
%Notice we only give waypoints in x, y, z. Roll, pitch, yaw needs to be
%close to 0.
% 设定无人机依次到达 [x, y, z] 的坐标点。
waypoints = [0 0 1;
             1 0 1;
             0 1 0;
             0 1 1;
             -1 0 1;
             0 -1 0;
             1 0 0];

% waypoints = [0 0 1;
%              1 0 1;
%              0 1 0
%              -1 0 0
%              0 -1 1
%              1 1 1
%              -1 1 0
%              -1 -1 0
%              2 1 1
%              2 -1 1];

% waypoints = [0 0 1;
%              1 0 1;
%              0 1 0
%              -1 0 0
%              0 -1 1
%              1 0 1];

% waypoints = [0 0 1;
%              1 0 1;
%              0 1 1
%              -1 0 1
%              0 -1 1
%              1 0 1];


% 仿真时间 t = 120 秒
t = 120; %time length of simulation
% 步长 dt = 0.01 秒
dt = 0.01;
%generate time vector
t_ref = 0:dt:t; 
%Get x, y, z reference from waypoints
x = waypoints(:,1);
y = waypoints(:,2);
z = waypoints(:,3);
%Make x,y,z reference to be same length as time
duration_each = floor(size(t_ref,2) / (size(waypoints,1)));
x_ref = repelem(x, duration_each);
y_ref = repelem(y, duration_each);
z_ref = repelem(z, duration_each);
if length(x_ref) < size(t_ref,2)
    x_ref(size(t_ref,2)) = x_ref(end);
    y_ref(size(t_ref,2)) = y_ref(end);
    z_ref(size(t_ref,2)) = z_ref(end);
end

%Combine time and reference
x_ref = [t_ref', x_ref];
y_ref = [t_ref', y_ref];
z_ref = [t_ref', z_ref];

%Run the simulink simulation (Need to finish the TODOs in the simulink before running the simulation!)
out = sim('Quadcopter_simulation_basic',t);

%Get output states
time = out.states.Time;

% 读取位置信息 x_out, y_out, z_out
x_out = out.states.Data(:,1);
y_out = out.states.Data(:,2);
z_out = out.states.Data(:,3);
% x_dot_out = out.states.Data(:,4);
% y_dot_out = out.states.Data(:,5);
% z_dot_out = out.states.Data(:,6);

% 读取姿态信息 phi_out, theta_out, psi_out
phi_out = out.states.Data(:,7);
theta_out = out.states.Data(:,8);
psi_out = out.states.Data(:,9);
% phi_dot_out = out.states.Data(:,10);
% theta_dot_out = out.states.Data(:,11);
% psi_dot_out = out.states.Data(:,12);

%Plot the X,Y,Z states.
% 第一部分：绘制 X, Y, Z 位置曲线
figure(1);
subplot(3,1,1);hold on;
plot(time,x_out); plot(time,x_ref(:,2));
title ('X to time'); ylabel('X');
subplot(3,1,2);hold on;
plot(time,y_out); plot(time,y_ref(:,2));
title ('Y to time'); ylabel('Y');
subplot(3,1,3);hold on;
plot(time,z_out); plot(time,z_ref(:,2));
title ('Z to time'); ylabel('Z');
xlabel('Time');
legend('State','Reference');

%Plot the roll, pitch, yaw states.
% 第二部分：绘制姿态角 (Roll, Pitch, Yaw) 随时间变化曲线
figure(2);
subplot(3,1,1);plot(time,phi_out); title ('phi to time'); ylabel('phi');
subplot(3,1,2);plot(time,theta_out); title ('theta to time'); ylabel('theta');
subplot(3,1,3);plot(time,psi_out); title ('psi to time'); ylabel('psi');
xlabel('Time');

%%%%%%%%%%%%%%%%%%%%%%%
% 不同颜色代表不同数据：蓝色（实际飞行）、红色（航点）、绿色（理论轨迹）。
figure(3);
plot3(x_out,y_out,z_out,'-b');
hold on
plot3(x,y,z,'*r');
plot3(x,y,z,'-g');
legend({'Quadcopter','Waypoints','Reference Trajectory'});
% some axis properties:
box on
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Position');

%get control usage
% u_T 代表总推力(T, Thrust):控制无人机升降高度。
u_T = out.control.Data(:,1);
% u_phi 代表滚转方向的力矩（torque phi）：控制横滚（Roll）运动。
u_phi = out.control.Data(:,2);
% u_theta 代表俯仰方向的力矩（torque theta）：控制俯仰（Pitch）运动。
u_theta = out.control.Data(:,3);
% u_psi 代表偏航方向的力矩（torque psi）：控制偏航（Yaw）运动。
u_psi = out.control.Data(:,4);
figure(4);
% 在同一张图上绘制 推力（u_T）、滚转力矩（u_phi）、俯仰力矩（u_theta） 和 偏航力矩（u_psi） 随时间变化的曲线。
plot(time,u_T, time,u_phi,time,u_theta, time, u_psi)
title('Control usage');
legend('T','torque phi','torque theta','torque psi');
