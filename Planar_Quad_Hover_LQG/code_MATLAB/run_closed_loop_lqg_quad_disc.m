% Discrete-time LQG simulation of the planar quad

clear; clc; close all;
run params_quad.m;               % load par

% 1) Linearize & discretize
[Ac,Bc,Cc,Dc] = linearize_quad(par);
sysc  = ss(Ac,Bc,Cc,Dc);
sysd  = c2d(sysc, par.dt);
[Ad,Bd,Cd,Dd] = ssdata(sysd);

% 2) Design discrete LQR
Qd  = eye(6);               
Rd  = 0.1*eye(2);           
Kd  = dlqr(Ad, Bd, Qd, Rd);

% 3) Design discrete Kalman filter
%   continuous noise covariances
Wc = diag([0.1,0.1,0.01,20,0.1,0.01]); % bigger variance on velocity in x cause of wind
Vc = diag([0.5,0.5,0.01]);
Wd = Wc * par.dt;            % approximate discrete process noise
Vd = Vc;                     % measurements at the sample instants
[Ld,~,~] = dlqe(Ad, eye(6), Cd, Wd, Vd);

% 4) Simulate
t   = 0:par.dt:par.tEnd;
N   = numel(t);
X   = zeros(6,N);    % true state
Xh  = zeros(6,N);    % estimated state
Y   = zeros(3,N);
U   = zeros(2,N);

% initial condition in deviation coords
X(:,1)  = [0; 0; par.theta0; 0; 0; 0];
Xh(:,1) = X(:,1);

for k = 2:N
  % ----- measurement (with noise) -----
  Y(:,k-1) = Cd*X(:,k-1) + mvnrnd(zeros(3,1), Vd)';
  
  % ----- control law (based on estimated state) -----
  U(:,k-1) = -Kd * Xh(:,k-1);
  
  % ----- plant update (disturb with discretized wind) -----
  aw = wind_model_quad(t(k-1), par);
  Wd_vec = [0;0;0; aw*par.dt; 0;0];  
  X(:,k) = Ad*X(:,k-1) + Bd*U(:,k-1) + Wd_vec;
  
  % ----- estimator update -----
  Xh(:,k) = Ad*Xh(:,k-1) + Bd*U(:,k-1) + Ld*(Y(:,k-1) - Cd*Xh(:,k-1));
end
Y(:,N) = Cd*X(:,N);
U(:,N) = -Kd * Xh(:,N);

% 5) Plot results
stateNames = {'x (m)','z (m)','theta (rad)','x\_dot (m/s)','z\_dot (m/s)','theta\_dot (rad/s)'};
figure;
for i = 1:6
  subplot(3,2,i);
  plot(t,  X(i,:), '-', 'LineWidth',1.5); hold on;
  plot(t, Xh(i,:), '--','LineWidth',1);
  title(stateNames{i}); grid on;
  xlabel('Time (s)'); ylabel(stateNames{i});
  legend('True','Est');
end

figure;
subplot(2,1,1);
plot(t, U(1,:), 'LineWidth',1.5);
title('Control: deltaF (N)'); xlabel('Time (s)'); ylabel('δF'); grid on;
subplot(2,1,2);
plot(t, U(2,:), 'LineWidth',1.5);
title('Control: tau (N·m)'); xlabel('Time (s)'); ylabel('\tau'); grid on;
