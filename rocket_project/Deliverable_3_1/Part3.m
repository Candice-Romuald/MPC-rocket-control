%% Part 3 Design MPC controller for each sub-system
addpath(fullfile('..', 'src'));
addpath('..\..\soft')

%define the subsystems
Ts = 1/20;
rocket = Rocket(Ts);

[xs,us] = rocket.trim();
sys = rocket.linearize(xs,us);

[sys_x, sys_y, sys_z,sys_roll] = rocket.decompose(sys,xs,us);

%% Todo 3.1

%% x controller
close all;

H = 5; %horizon in s
mpc_x = MPC_Control_x(sys_x, Ts, H);

%ux = mpc_x.get_u([0;0;0;0])

%simulate
x0 = [0;0;0;5];
Tf = 8;
[Tx,X_subx,U_subx] = rocket.simulate(sys_x,x0,Tf,@mpc_x.get_u,0);
phx = rocket.plotvis_sub(Tx,X_subx,U_subx,sys_x,xs,us);

%% y controller
close all;

H = 8; %horizon in s
mpc_y = MPC_Control_y(sys_y, Ts, H);

%simulate
y0 = [0;0;0;5];
Tf = 10;
[Ty,X_suby,U_suby] = rocket.simulate(sys_y,y0,Tf,@mpc_y.get_u,0);

phy = rocket.plotvis_sub(Ty,X_suby,U_suby,sys_y,xs,us);


%% z controller
%close all;

H = 4; %horizon in s
mpc_z = MPC_Control_z(sys_z, Ts, H);
uz = mpc_z.get_u([0;10]);

%simulate
z0 = [0;5];
Tf = 8;
[Tz,X_subz,U_subz] = rocket.simulate(sys_z,z0,Tf,@mpc_z.get_u,0);
phz = rocket.plotvis_sub(Tz,X_subz,U_subz,sys_z,xs,us);


%% roll controller
close all;

H = 5; %horizon in s
mpc_r = MPC_Control_roll(sys_roll, Ts, H);

%simulate
x0 = [0; deg2rad(45)];
Tf = 10;
[Tr,X_subr,U_subr] = rocket.simulate(sys_roll,x0,Tf,@mpc_r.get_u,0);
phr = rocket.plotvis_sub(Tr,X_subr,U_subr,sys_roll,xs,us);




