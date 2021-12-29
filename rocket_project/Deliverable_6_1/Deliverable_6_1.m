addpath(fullfile('..', 'src'));
addpath('..\..\soft')
%% TODO: This file should produce all the plots for the deliverable

%Simulation with default roll angle 

Ts = 1/10;
rocket = Rocket(Ts);

H = 1;
nmpc = NMPC_Control(rocket,H);

Tf = 30;
ref = @(t_,x_) rocket.MPC_ref(t_,Tf);
x0 = zeros(12,1);

[T,X,U,Ref] = rocket.simulate_f(x0,Tf,nmpc,ref);
rocket.anim_rate = 2;
ph = rocket.plotvis(T,X,U,Ref);
ph.fig.Name = 'Non linear MPC, roll normal';

%% Simulation of non linear MPC with degenerated roll angle 

Ts = 1/10;
rocket = Rocket(Ts);

H = 1;
nmpc = NMPC_Control(rocket,H);

Tf = 30;
roll_max = deg2rad(50);
ref = @(t_,x_) rocket.MPC_ref(t_,Tf, roll_max);
x0 = zeros(12,1);

[T,X,U,Ref] = rocket.simulate_f(x0,Tf,nmpc,ref);
rocket.anim_rate = 2;
ph = rocket.plotvis(T,X,U,Ref);
ph.fig.Name = 'Non linear MPC with critical roll angle';


%% Simulation of linear MPC with degenerated roll angle 
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
roll_max = deg2rad(50);
Tf = 30;
ref = @(t_,x_) rocket.MPC_ref(t_,Tf,roll_max);
x0 = zeros(12,1); %initial state

[T,X,U,Ref]=rocket.simulate_f(x0,Tf,mpc,ref);
rocket.anim_rate = 10; %make animation faster
ph = rocket.plotvis(T,X,U,Ref);
ph.fig.Name = 'Merged lin. MPC in non linear simulation with critical roll angle';



