%% TODO: This file should produce all the plots for the deliverable
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

% merge controllers
mpc = rocket.merge_lin_controllers(xs,us,mpc_x,mpc_y,mpc_z,mpc_roll);

%set up reference function: path to track
Tf = 30;
ref = @(t_,x_) rocket.MPC_ref(t_,Tf);

x0 = zeros(12,1); %initial state

[T,X,U,Ref]=rocket.simulate_f(x0,Tf,mpc,ref);
rocket.anim_rate = 2; %make animation faster
ph = rocket.plotvis(T,X,U,Ref);
ph.fig.Name = 'Merged lin. MPC in non linear simulation';


