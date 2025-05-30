%% Data processing and ploting
Euler_angles_file = load("Euler_angles.mat");
Euler_angles = Euler_angles_file.values.Data;
times = Euler_angles_file.values.Time;
Angular_vels_file = load("Angular_velocity.mat");
omega = Angular_vels_file.values.Data;
Torques_file = load("Torques.mat");
Torques = Torques_file.values.Data;

reference_file = load("Reference_commands.mat");
references = reference_file.values.Data;

Angular_vels_file = load("Angular_velocity_Euler.mat");
omega_Euler = Angular_vels_file.values.Data;

Euler_angles_file = load("Euler_angles_Euler.mat");
Euler_angles_Euler = Euler_angles_file.values.Data;

%% Ploting
% Ploting Euler angles as a function of time
fig = figure;

plot(times, Euler_angles, 'LineStyle','-.')
grid()
ylabel('Euler angles [deg.]')
xlabel('Time past first Epoch [s]')
legend('Yaw \psi','Pitch \theta', 'Roll \phi','Location','eastoutside')

saveas(fig,'euler_angles.png')

fig = figure;
fig.Position(3:4) = [1200 400];  % [width, height] in pixels
% Use tiledlayout for tighter control of spacing
t = tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile
plot(times, Euler_angles(:,1),times,reshape(references(1,1,:),[],1)/deg)
grid on
ylabel('Yaw \psi [deg.]')
xlabel('Time past first Epoch [s]')
legend('Simulation','reference')

nexttile
plot(times, Euler_angles(:,2),times,reshape(references(1,2,:),[],1)/deg)
grid on
ylabel('Pitch \theta [deg.]')
xlabel('Time past first Epoch [s]')
legend('Simulation','reference')

nexttile
plot(times, Euler_angles(:,3),times,reshape(references(1,3,:),[],1)/deg)
grid on
ylabel('Roll \phi [deg.]')
xlabel('Time past first Epoch [s]')
legend('Simulation','reference')

% Save figure
saveas(fig, 'euler_angles3.png')

fig = figure;
fig.Position(3:4) = [1200 400];  % [width, height] in pixels
% Use tiledlayout for tighter control of spacing
t = tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile
plot(times, -Euler_angles(:,1)+reshape(references(1,1,:),[],1)/deg)
grid on
ylabel('Yaw error \psi [deg.]')
xlabel('Time past first Epoch [s]')


nexttile
plot(times, -Euler_angles(:,2)+reshape(references(1,2,:),[],1)/deg)
grid on
ylabel('Pitch error \theta [deg.]')
xlabel('Time past first Epoch [s]')


nexttile
plot(times, -Euler_angles(:,3)+reshape(references(1,3,:),[],1)/deg)
grid on
ylabel('Roll error \phi [deg.]')
xlabel('Time past first Epoch [s]')

% Save figure
saveas(fig, 'euler_angles3.png')


% Ploting angular velocity in the body-fixed frame as a function of time 
fig = figure;
plot(times, omega,'LineStyle','-.')
xlabel('Time past first Epoch [s]')
ylabel('Body-fixed angular velocity [deg./s]')
legend('\omega_x','\omega_y', '\omega_z')

saveas(fig, 'omega.png')
%% Performance plots
% Plot Euler angles, angular velocity, and torques in one tiled layout
fig = figure;

t = tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
fig.Position(3:4) = [1200 400];  % [width, height] in pixels

% ------------------ Tile 1: Euler Angles + Single Reference ------------------
nexttile
hold on
plot(times, Euler_angles(:,1), 'b-', 'DisplayName','Yaw \psi')
plot(times, Euler_angles(:,2), 'r-', 'DisplayName','Pitch \theta')
plot(times, Euler_angles(:,3), 'g-', 'DisplayName','Roll \phi')

% Extract one reference vector (any channel, since all are the same)
ref_deg = reshape(references(1,1,:), [], 1) / deg;
plot(times, ref_deg, 'k--', 'LineWidth', 1.5, 'DisplayName','Reference')
hold off

grid on
xlabel('Time past first Epoch [s]')
ylabel('Euler Angles [deg.]')
legend()
title('Euler Angles and Reference Command')

% ------------------ Tile 2: Angular Velocities ------------------
nexttile
plot(times, omega, 'LineStyle','-')
grid on
xlabel('Time past first Epoch [s]')
ylabel('Angular Velocity [deg./s]')
legend('\omega_x','\omega_y','\omega_z')
title('Body-Fixed Angular Velocities')

% ------------------ Tile 3: Torques ------------------
nexttile
data = Torques(1:length(times),:)
plot(times, data, 'LineStyle','-')
grid on
xlabel('Time past first Epoch [s]')
ylabel('Torque [N·m]')
legend('\tau_x','\tau_y','\tau_z')
title('Control Torques in Body Frame')

% Add 10% margin to y-axis
ymin = min(data(:));
ymax = max(data(:));
yrange = ymax - ymin;
margin = 0.1 * yrange;
ylim([ymin - margin, ymax + margin])

% Save figure
saveas(fig, 'euler_vel_torque_summary.png')

%% Performance metrics
% Run loading block befor this:
init = 30*ones(3,1);
target = 0*ones(3,1);
t = times(1:1000);
y = Euler_angles(1:1000,:);
for i = 1:3
    S = stepinfo(y(:,i), t, target(i), init(i))
    metrics(1,i) = S.RiseTime
    metrics(2,i) = S.PeakTime
    metrics(3,i) = S.Overshoot
    metrics(4,i) = S.SettlingTime
end

init = 0*ones(3,1);
target = 60*ones(3,1);
t = times(1001:5001) - times(1001);
y = Euler_angles(1001:5001,:);
for i = 1:3
    S = stepinfo(y(:,i), t, target(i), init(i))
    metrics(1,i+3) = S.RiseTime
    metrics(2,i+3) = S.PeakTime
    metrics(3,i+3) = S.Overshoot
    metrics(4,i+3) = S.SettlingTime
end

init = 60*ones(3,1);
target = -60*ones(3,1);
t = times(5002:9001) - times(5002);
y = Euler_angles(5002:9001,:);
for i = 1:3
    S = stepinfo(y(:,i), t, target(i), init(i))
    metrics(1,i+6) = S.RiseTime
    metrics(2,i+6) = S.PeakTime
    metrics(3,i+6) = S.Overshoot
    metrics(4,i+6) = S.SettlingTime
end

init = -60*ones(3,1);
target = 0*ones(3,1);
t = times(9002:15001)- times(9002);
y = Euler_angles(9002:15001,:);
for i = 1:3
    S = stepinfo(y(:,i), t, target(i), init(i))
    metrics(1,i+9) = S.RiseTime
    metrics(2,i+9) = S.PeakTime
    metrics(3,i+9) = S.Overshoot
    metrics(4,i+9) = S.SettlingTime
end


%% Code to easily generate table containing performance metrics


% === Labels ===
metric_names = {
    'Rise Time [s]'
    'Peak Time [s]'
    'Overshoot [\%]'
    'Settling Time [s]'
};

time_ranges = {
    '0 - 99.9 [s]'
    '100 - 500 [s]'
    '500.1 - 900 [s]'
    '900.1 - 1500 [s]'
};

euler_labels = {'$\psi$', '$\theta$', '$\phi$'};

% === Generate LaTeX code ===
fprintf('\\begin{table}[ht]\n');
fprintf('\\centering\n');
fprintf('\\renewcommand{\\arraystretch}{1.2}\n');
fprintf('\\begin{tabular}{|c');

for i = 1:4
    fprintf('|c|c|c');
end
fprintf('|}\n');

% Header row 1
fprintf('\\hline\n');
fprintf('\\multirow{2}{*}{Metric} ');
for i = 1:4
    fprintf('& \\multicolumn{3}{|c|}{%s} ', time_ranges{i});
end
fprintf('\\\\\\cline{2-13}\n');

% Header row 2
for i = 1:4
    for j = 1:3
        if i == 1 && j == 1
            fprintf('& %s ', euler_labels{j});
        else
            fprintf('& %s ', euler_labels{j});
        end
    end
end
fprintf('\\\\\n\\hline\n');

% Data rows
for i = 1:4
    fprintf('%s ', metric_names{i});
    for j = 1:12
        fprintf('& %.1f ', metrics(i,j));
    end
    fprintf('\\\\\\hline\n');
end

% End table
fprintf('\\end{tabular}\n');
fprintf('\\caption{Performance metrics of the controller for each reference command and Euler angle}\n');
fprintf('\\label{tab:control_metrics}\n');
fprintf('\\end{table}\n');


% Difference in response between Euler and Quaternion based models
%{
fig = figure;
plot(times, abs(omega-omega_Euler),'LineStyle','-.')
xlabel('Time past first Epoch [s]')
ylabel('Angular velocity difference [deg./s]')
legend('\omega_x','\omega_y', '\omega_z')

saveas(fig, 'omega_comp.png')

fig = figure;

plot(times, abs(Euler_angles- Euler_angles_Euler), 'LineStyle','-.')
grid()
ylabel('Euler angles difference [deg.]')
xlabel('Time past first Epoch [s]')
legend('Yaw \psi','Pitch \theta', 'Roll \phi','Location','eastoutside')

saveas(fig,'euler_angles_comp.png')
%}