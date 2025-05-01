% plot_cpp_results.m
% Reads data.csv (from the C++ sim) and plots true vs. estimated states.

% Read the CSV (skips header automatically)
M = readmatrix('data.csv');

% Columns:
% 1 = step, 2 = time, 3–8 = x0..x5, 9–14 = xhat0..xhat5
t     = M(:,2);
X     = M(:,3:8);
Xhat  = M(:,9:14);

% State names for plotting
stateNames = {'x (m)','z (m)','theta (rad)','x\_dot (m/s)','z\_dot (m/s)','theta\_dot (rad/s)'};

% Plot
figure;
for i = 1:6
    subplot(3,2,i);
    plot(t, X(:,i), '-', 'LineWidth', 1.5); hold on;
    plot(t, Xhat(:,i), '--', 'LineWidth', 1.2);
    xlabel('Time (s)');
    ylabel(stateNames{i});
    title(stateNames{i});
    legend('True','Estimated','Location','Best');
    grid on;
end