%% AE4313 - Spacecraft properties file
mu_Earth = 398600.4418e9% [m^3/s^2] - Earth's gravitational parameter
deg = pi/180 % Convertion factor from degrees to radians
h = 7e5 % [m] - Orbital altitude
R_e = 6378e3 % [m] - Earth Radius
r = R_e + h
n = (mu_Earth/r^3).^0.5% [s^-1] - Orbital angular velocity
J11 = 124.531;
J22 = 124.586;
J33 = 1.704;
J = diag([J11,J22,J33])% [kg m^2] - Tensor of inertia
t_d = [1e-4,1e-4,1e-4]% [N m^2] - Disturbance torque
initial_euler_angles =[30,30,30].*deg% Initial Euler angles
init_q = eul2quat(initial_euler_angles,"ZYX")% Inital quaternion
w = [0,0,0].*deg% [deg/s] Initial angular velocity in the
sim_time = 1500 % [s] - Simulation duration
T = 2*pi/n % [s] - Orbital period 

% Attitude commands:
com_0 = [0,0,0];
com_1 = [60,60,60]*deg;
com_2 = [-60,-60,-60]*deg;
com_3 = [0,0,0]*deg;
