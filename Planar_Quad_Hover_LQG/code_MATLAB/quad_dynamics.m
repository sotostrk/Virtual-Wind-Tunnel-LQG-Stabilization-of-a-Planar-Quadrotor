% Nonlinear planar quadrotor dynamics: x, z, theta, velocities
% Inputs: u = [deltaF; tau] where deltaF is thrust deviation, tau is pitch torque
function dx = quad_dynamics(t, x, u, par)
    deltaF = u(1);
    tau    = u(2);
    F      = par.F + deltaF;
    theta  = x(3);
    aw     = wind_model_quad(t, par);
    Fx     = F * sin(theta);
    Fz     = F * cos(theta);
    xdd    = Fx/par.m + aw;
    zdd    = Fz/par.m - par.g;
    thetadd = tau/par.I;
    dx     = [ x(4);
               x(5);
               x(6);
               xdd;
               zdd;
               thetadd ];
end