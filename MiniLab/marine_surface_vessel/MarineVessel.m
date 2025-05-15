clear 
clc
close all

%% begin simulation
% simulation time set
T=100;
size=0.01;
t=0:size:T;
n=length(t);

%%%%%TODO_1%%%%%%
% feedforward gain(adjustable)



% desired trajectory(optional)
widthsin=0.1;
eta_d=10*[sin(widthsin*t);sin(widthsin*t);sin(widthsin*t)];
deta_d=10*[widthsin*cos(widthsin*t);widthsin*cos(widthsin*t);widthsin*cos(widthsin*t)];
ddeta_d=10*[-widthsin*widthsin*sin(widthsin*t);...
-widthsin*widthsin*sin(widthsin*t);...
-widthsin*widthsin*sin(widthsin*t)];

% eta error initialization
e=zeros(3,length(t));
de=zeros(3,length(t));

% backstepping error initialization
z1=[0;0;0];
dz1=[0;0;0];
z2=[0;0;0];
dz2=[0;0;0];

%% Mathmatical Model of the Fully Actuated Ocean Surface Vessel
eta=zeros(3,length(t)); % position (world frame)
deta=zeros(3,length(t)); %Velocity (world frame)
ddeta=zeros(3,length(t)); %acceleration (world frame)
v=zeros(3,length(t)); % velocity
dv=zeros(3,length(t)); % acceleration

M=zeros(3,3); % inertia matrix
C=zeros(3,3); % coriolis matrix
g=zeros(3,1); % unknown vector of restoring forces 
TauD=zeros(3,1); % damping term and disturbance term

Tau=zeros(3,length(t)); % the vector of input signals

% % Disturbance term=Output white Gaussian noise
% x=sin(t);
% Eb = mean(x.^2)*T;
% Fs = 0.004;
% Noise1 = sqrt(Eb*Fs/2)*randn(length(x),1);
% Noise2 = sqrt(Eb*Fs/2)*randn(length(x),1);
% Noise3 = sqrt(Eb*Fs/2)*randn(length(x),1);
% TauD=[Noise1(1);Noise2(1);Noise3(1)];

%% the first step
for i=2:n
% set mass and moment of inertia
mass = 5312200;
Iz = 3745400000;


%%%%%%TODO_2%%%%%%%
%Set Mass matrix


%set coriolis matrix


% set unknown vector of restoring forces 
g=[0;0;0];


%%%%%%%TODO_3%%%%%%%%
% set rotation transformation matrix



%%%%%%%TODO_4%%%%%%%%
% set backstepping error



%%%%%%%TODO_5%%%%%%%%
% set real time input signal



% calculate the next eta v & dv
dv(:,i)=M\(Tau(:,i)-C*v(:,i-1)-g);
v(:,i)=v(:,i-1)+size*dv(:,i-1);
ddeta(:,i)=dJ*v(:,i)+J*dv(:,i);
deta(:,i)=J*v(:,i);
eta(:,i)=eta(:,i-1)+size*deta(:,i-1)+1/2*size^2*ddeta(:,i-1);

end

% Tracking error
E=eta-eta_d;

% Plot error curve
plot(t,E);
xlabel('time');
ylabel('error');
title('Tracking error [rad]');
legend("error_1 in z_1","error_2 in z_1" ,"error_3 in z_1");