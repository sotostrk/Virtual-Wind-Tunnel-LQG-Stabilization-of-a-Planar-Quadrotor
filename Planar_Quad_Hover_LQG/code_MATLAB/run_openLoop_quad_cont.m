% Simulate and plot each of the 6 states to show continuous time open‐loop instability
clear; clc; close all;
run params_quad.m;    % loads 

t = 0:par.dt:par.tEnd;
X = zeros(6, numel(t));
X(:,1) = [0; 0; par.theta0; 0; 0; 0];  % [x; z; theta; xdot; zdot; thetadot]

for k = 2:numel(t)
    u = [par.F; 0];
    dx = quad_dynamics(t(k-1), X(:,k-1), u, par);
    X(:,k) = X(:,k-1) + par.dt * dx;
end

% Plot all 6 states 
figure;
stateNames = {'x (m)','z (m)','\theta (rad)','ẋ (m/s)','ż (m/s)','\thetȧ (rad/s)'};
for i = 1:6
    subplot(3,2,i);
    plot(t, X(i,:), 'LineWidth',1.5);
    xlabel('Time (s)');
    ylabel(stateNames{i});
    title(['Open‐Loop State: ' stateNames{i}]);
    grid on;
end