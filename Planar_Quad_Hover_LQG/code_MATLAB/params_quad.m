% Physical & simulation parameters
par.m         = 0.5;           % mass (kg)
par.g         = 9.81;          % gravity (m/s^2)
par.F         = par.m*par.g;   % hover thrust magnitude (N)
par.I         = 0.02;          % moment of inertia about pitch axis (kg*m^2)
% Visualization wind 
par.theta0    = 0.03;          % initial pitch bias (rad)
par.wind_amp  = 0.5;           % wind acceleration amplitude (m/s^2)
par.wind_freq = 1.0;           % wind oscillation frequency (rad/s)
% Sim settings
par.tEnd      = 10;            % simulation duration (s)
par.dt        = 0.02;          % time step (s)