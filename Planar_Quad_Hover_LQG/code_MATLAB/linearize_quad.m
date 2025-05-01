%% File: linearize_quad.m
% Linearize about theta=0, deltaF=0, tau=0
function [A,B,C,D] = linearize_quad(par)
    % States: [x; z; theta; xdot; zdot; thetadot]
    % Inputs: [deltaF; tau]
    A = zeros(6);
    A(1,4) = 1;
    A(2,5) = 1;
    A(3,6) = 1;
    A(4,3) = par.g;      % partial derivative of xddot wrt theta
    % B matrix
    B = zeros(6,2);
    B(5,1) = 1/par.m;    % deltaF affects vertical acceleration
    B(6,2) = 1/par.I;    % tau affects angular acceleration
    C = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 1 0 0 0];    % measure x, z, theta
    D = zeros(3,2);
end