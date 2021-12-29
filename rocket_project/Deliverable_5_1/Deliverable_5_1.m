%% TODO: This file should produce all the plots for the deliverable

%Simulate to see the effect of mass missmatch

%Setp up
addpath(fullfile('..', 'src'));
addpath('..\..\soft')
%define the subsystems
Ts = 1/20;
rocket = Rocket(Ts);
[xs,us] = rocket.trim();
sys = rocket.linearize(xs,us);
[sys_x, sys_y, sys_z,sys_roll] = rocket.decompose(sys,xs,us);
H = 5; %horizon in s
% x controller
mpc_x = MPC_Control_x(sys_x, Ts, H);
% y controller
mpc_y = MPC_Control_y(sys_y, Ts, H);
% z controller
mpc_z = MPC_Control_z(sys_z, Ts, H);
% Roll controller 
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);
%simulate without correction
mpc = rocket.merge_lin_controllers(xs,us,mpc_x,mpc_y,mpc_z,mpc_roll);

% set up reference function: path to track
Tf = 30;
ref = @(t_,x_) rocket.MPC_ref(t_,Tf);

x0 = zeros(12,1); %initial state

rocket.mass = 1.783;

[T,X,U,Ref]=rocket.simulate_f(x0,Tf,mpc,ref);
rocket.anim_rate = 5; %make animation faster
ph = rocket.plotvis(T,X,U,Ref);
ph.fig.Name = 'Merged lin. MPC in non linear simulation without offset tracking';

%% simulate with offset tracking
addpath(fullfile('..', 'src'));
addpath('..\..\soft')
%define the subsystems
Ts = 1/20;
rocket = Rocket(Ts);
[xs,us] = rocket.trim();
sys = rocket.linearize(xs,us);
[sys_x, sys_y, sys_z,sys_roll] = rocket.decompose(sys,xs,us);
H = 5; %horizon in s
% x controller
mpc_x = MPC_Control_x(sys_x, Ts, H);
% y controller
mpc_y = MPC_Control_y(sys_y, Ts, H);
% z controller
mpc_z = MPC_Control_z(sys_z, Ts, H);
% Roll controller 
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);
%simulate without correction
mpc = rocket.merge_lin_controllers(xs,us,mpc_x,mpc_y,mpc_z,mpc_roll);

% set up reference function: path to track
Tf = 30;
ref = @(t_,x_) rocket.MPC_ref(t_,Tf);

x0 = zeros(12,1); %initial state

rocket.mass = 1.783;

[T,X,U,Ref,Z_hat]=rocket.simulate_f_est_z(x0,Tf,mpc,ref,mpc_z,sys_z);
rocket.anim_rate = 20; %make animation faster
ph = rocket.plotvis(T,X,U,Ref);
ph.fig.Name = 'Merged lin. MPC in non linear simulation with offset tracking';


% compare the estimate and the real state

figure
plot(X(12,:)-Z_hat(12,:))
xlabel('iteration'); ylabel('z state estimate error')
title('Z estimattion state error')
figure
plot(X(9,:)-Z_hat(9,:))
xlabel('iteration'); ylabel('Vz state estimate error')
title('Vz estimattion state error')
figure
plot(Z_hat(13,:))
xlabel('iteration'); ylabel('disturbance estimate')
title('disturbance estimate')

