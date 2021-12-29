%% TODO: This file should produce all the plots for the deliverable

addpath(fullfile('..', 'src'));
addpath('..\..\soft')

%define the subsystems
Ts = 1/20;
rocket = Rocket(Ts);

[xs,us] = rocket.trim();
sys = rocket.linearize(xs,us);

[sys_x, sys_y, sys_z,sys_roll] = rocket.decompose(sys,xs,us)

%% x controller
%close all;

H = 5; %horizon in sc
mpc_x = MPC_Control_x(sys_x, Ts, H);

%simulate
x0 = [0;0;0;0];
Tf = 8;

x_ref = -5; %its r (= y the output of the system, in this system it is the pisition x lol) the reference to track
[Tx,X_subx,U_subx] = rocket.simulate(sys_x,x0,Tf,@mpc_x.get_u,x_ref);
phx = rocket.plotvis_sub(Tx,X_subx,U_subx,sys_x,xs,us,x_ref);


%% y controller
%close all;

H = 10; %horizon in sc
mpc_y = MPC_Control_y(sys_y, Ts, H);

%simulate
x0 = [0;0;0;0];
Tf = 8;

y_ref = 5; %its r (= y the output of the system, in this system it is the pisition y lol) the reference to track
[Ty,X_suby,U_suby] = rocket.simulate(sys_y,x0,Tf,@mpc_y.get_u,y_ref);
phy = rocket.plotvis_sub(Ty,X_suby,U_suby,sys_y,xs,us,y_ref);


%% z controller
%close all;

H = 10; %horizon in sc
mpc_z = MPC_Control_z(sys_z, Ts, H);

%simulate
x0 = [0;0];
Tf = 15;

z_ref = -5; %its r (= y the output of the system, in this system it is the pisition z lol) the reference to track
[Tz,X_subz,U_subz] = rocket.simulate(sys_z,x0,Tf,@mpc_z.get_u,z_ref);
phy = rocket.plotvis_sub(Tz,X_subz,U_subz,sys_z,xs,us,z_ref);

%% Roll controller 
%close all;

H = 4; %horizon in sc
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

%simulate
x0 = [0;0];
Tf = 10;

roll_ref = pi/4; %its r (= y the output of the system, in this system it is the pisition z lol) the reference to track
[Tr,X_subr,U_subr] = rocket.simulate(sys_roll,x0,Tf,@mpc_roll.get_u,roll_ref);
phr = rocket.plotvis_sub(Tr,X_subr,U_subr,sys_roll,xs,us,roll_ref);





