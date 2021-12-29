%% todo 1
Ts = 1/20;
rocket = Rocket(Ts);
u = [0,0,0.5,0]';
[b_f,b_M] = rocket.getForceAndMomentFromThrust(u)

x = zeros(12,1);
x_dot = rocket.f(x,u)

%% todo 1.2
rocket = Rocket(Ts);
Tf = 2.0;

x0 = [deg2rad([2 -1 0, -2 2 0]),0 0 0, 0 0 0]';
u = [deg2rad([2 0]), 60, 0]';
[T,X,U]= rocket.simulate_f(x0,Tf,u);

rocket.anim_rate = 1.0;
rocket.vis(T,X,U);

%% explore rocketbehaviour

%control ascending or descending
rocket = Rocket(Ts);
Tf = 4.0;


x0 = [deg2rad([0 0 0, 0 0 0]),0 0 0, 0 0 0]';
u = [deg2rad([0 0]), 58, 0]'; %57 58 c'est la limite pour ne pas drop...
[T,X,U]= rocket.simulate_f(x0,Tf,u);

rocket.anim_rate = 1.0;
rocket.vis(T,X,U);

%% vertical rotating
rocket = Rocket(Ts);
Tf = 2.0;

x0 = [deg2rad([0 0 0, 0 0 0]),0 0 0, 0 0 0]';
u = [deg2rad([0 0]), 57, 20]'; %+-20 for rotating rocket in z axis.
[T,X,U]= rocket.simulate_f(x0,Tf,u);

rocket.anim_rate = 1.0;
rocket.vis(T,X,U);

%% rotating x or y
rocket = Rocket(Ts);
Tf = 2.0;
%change delta 1 or 2.
delta1 = 2;
delta2 = 0;
x0 = [deg2rad([0 0 0, 0 0 0]),0 0 0, 0 0 0]';
u = [deg2rad([delta1 delta2]), 57, 0]'; 
[T,X,U]= rocket.simulate_f(x0,Tf,u);

rocket.anim_rate = 1.0;
rocket.vis(T,X,U);


%% try flying along x or y axis;  hard
rocket = Rocket(Ts);
Tf = 2.0;
%change delta 1 or 2.
delta1 = -2;
delta2 = 0;
x0 = [deg2rad([0 0 0, -90 0 0]),0 10 0, 0 0 0]';
u = [deg2rad([delta1 delta2]), 57, 0]'; 
[T,X,U]= rocket.simulate_f(x0,Tf,u);

rocket.anim_rate = 1.0;
rocket.vis(T,X,U);

%% todo 2.1 2.2

Ts = 1/20;
rocket = Rocket(Ts);

[xs,us] = rocket.trim()
sys = rocket.linearize(xs,us)

[sys_x, sys_y, sys_z,sys_roll] = rocket.decompose(sys,xs,us)
