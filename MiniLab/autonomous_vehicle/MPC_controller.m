function y = MPC_controller(x)
global Ty M Izz lf lr dt xx vx N X
%%%%%%%  input variables
vy=x(1);                  % lateral velocity£¬ m/s
vx=x(2);                  % longitudinal velocity, m/s
fy=x(3);                  % Yaw angle£¬rad
wr=x(4);                  % Yaw rate rad/s
Y=x(5);                   % Y position£¬ m
X=x(6);                   % X position£¬m
t=x(7);                   % Time, s
% intalization
dt=0.05;                  % Discrete timestep, 
Nx=4;                     % number of state variable
% vehicle parameters
M=1270;                   % vehicle mass, kg
Izz=1536.7;               % moment of inertia of the vehicle
lf=1.015;                 % distance from the center of gravity to the front axles, m
lr=1.895;                 % distance from the center of gravity to the front axles, m
% before 2s lane-keeping with fix predictive horizon(1s), after 2s change lane
% in 3s with receding predictive horizon, after 5s lane-keeping with fix predictive horizon (1s)
T_lane_keep=1;
if t<2
    tf=T_lane_keep;
    Ty=0;
elseif t<5
    tf=5-t;
    Ty=4;
else
    tf=T_lane_keep;
    Ty=4;
end

N=max(round(tf/dt),round(T_lane_keep/dt)); %predictive horizon points
xx=zeros(Nx,1);
xx(1,1)=Y; xx(2,1)=fy; xx(3,1)=vy; xx(4,1)= wr;
%-----------------------------------------simulation start
%%%%%%%%%%%%TODO_3%%%%%%%%%%%%%%%


%%%%%%%%%%%TODO_3 end%%%%%%%%%%
end


%%

function [c,ceq]=mycon1(u)
%%%%%%%%%%%TODO_1%%%%%%%%%%%%


%%%%%%%%%%TODO_1_end%%%%%%%%%
end

function f=myobj1(u)
%%%%%%%%%TODO_2%%%%%%%%%%%%%%%%


%%%%%%%%%%TODO_2_end%%%%%%%%%%%%%

end



