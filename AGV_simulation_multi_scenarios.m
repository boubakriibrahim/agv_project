%% AGV_simulation_multi_scenarios.m
% This MATLAB script simulates an AGV with multiple test scenarios and publishes
% its state as ROS2 nav_msgs/msg/Odometry messages for integration with Gazebo.
% All options are activated.

%% -----------------------
%% 1. Initial Configuration
%% -----------------------

clear; clc; close all;

% AGV physical parameters
params.m    = 150;       % mass (kg)
params.Iz   = 120;       % moment of inertia (kg·m^2)
params.a    = 1.2;       % distance from CG to front axle (m)
params.b    = 0.8;       % distance from CG to rear axle (m)
params.L    = params.a + params.b;  % wheelbase (m)
params.Cf   = 50000;     % front cornering stiffness (N/rad)
params.Cr   = 50000;     % rear cornering stiffness (N/rad)
params.mu   = 0.8;       % friction coefficient
params.g    = 9.81;      % gravitational acceleration (m/s^2)
params.width= 1.5;       % vehicle width (m)

% Initial state [x, y, theta, v, delta]
init_state = [0; 0; 0; 0; 0];

% Simulation time
tspan = [0 10];      % seconds
dt    = 0.01;        % time step (s)
t_sim = tspan(1):dt:tspan(2);

%% -----------------------
%% 2. Test Scenario Selection and Command Definition
%% -----------------------

% Choose one scenario:
%   1 - Straight line
%   2 - Circular motion
%   3 - Sinusoidal trajectory
%   4 - External perturbation (noisy commands)
scenario = 3;  % Change this value to 1, 2, 3, or 4

% Initialize control input matrix u (2 x length(t_sim))
% u(1,:) = acceleration (m/s^2), u(2,:) = steering rate (rad/s)
u = zeros(2, length(t_sim));

switch scenario
    case 1  % Straight line
        for i = 1:length(t_sim)
            if t_sim(i) < 5
                u(1,i) = 0.5;
                u(2,i) = 0;
            else
                u(1,i) = 0;
                u(2,i) = 0;
            end
        end
        
    case 2  % Circular motion
        for i = 1:length(t_sim)
            u(1,i) = 0.3;
            u(2,i) = 0.1;
        end
        
    case 3  % Sinusoidal trajectory
        for i = 1:length(t_sim)
            u(1,i) = 0.3;
            u(2,i) = 0.2 * sin(0.5*t_sim(i));
        end
        
    case 4  % External perturbation with noise
        rng(1); % reproducibility
        for i = 1:length(t_sim)
            if t_sim(i) < 5
                u(1,i) = 0.5 + 0.05*randn;
                u(2,i) = 0.1 + 0.02*randn;
            else
                u(1,i) = 0.2 + 0.05*randn;
                u(2,i) = -0.2 + 0.02*randn;
            end
        end
    otherwise
        error('Scenario not recognized. Choose 1, 2, 3, or 4.');
end

%% -----------------------
%% 3. Simulation of the Kinematic Model
%% -----------------------

[t_out, x_out] = ode45(@(t,x) agv_kinematic_model(t,x,u,params,t_sim), tspan, init_state);

%% -----------------------
%% 4. Analysis and Visualization
%% -----------------------

% Plot state variables
figure;
subplot(2,2,1);
plot(x_out(:,1), x_out(:,2), 'b-', 'LineWidth',2);
title('AGV Trajectory');
xlabel('X (m)'); ylabel('Y (m)'); grid on;

subplot(2,2,2);
plot(t_out, x_out(:,3), 'r-', 'LineWidth',2);
title('Orientation (\theta)');
xlabel('Time (s)'); ylabel('\theta (rad)'); grid on;

subplot(2,2,3);
plot(t_out, x_out(:,4), 'g-', 'LineWidth',2);
title('Velocity (v)');
xlabel('Time (s)'); ylabel('v (m/s)'); grid on;

subplot(2,2,4);
plot(t_out, x_out(:,5), 'm-', 'LineWidth',2);
title('Steering Angle (\delta)');
xlabel('Time (s)'); ylabel('\delta (rad)'); grid on;

% For straight-line scenario, calculate and display tracking error
if scenario == 1
    v_avg = 0.5 * 5;
    x_ref = linspace(0, v_avg*tspan(2), length(t_out));
    y_ref = zeros(1, length(t_out));
    erreur = sqrt((x_out(:,1) - x_ref').^2 + (x_out(:,2) - y_ref').^2);
    erreur_moyenne = mean(erreur);
    
    figure;
    plot(x_out(:,1), x_out(:,2), 'b-', 'LineWidth',2); hold on;
    plot(x_ref, y_ref, 'k--', 'LineWidth',2);
    title('Simulated Trajectory vs Reference');
    xlabel('X (m)'); ylabel('Y (m)');
    legend('Simulation','Reference'); grid on; hold off;
    
    figure;
    plot(t_out, erreur, 'k-', 'LineWidth',2);
    title(['Tracking Error (avg = ', num2str(erreur_moyenne, '%.3f'), ' m)']);
    xlabel('Time (s)'); ylabel('Error (m)'); grid on;
end

% Dynamic visualization of AGV along the trajectory
figure;
plot(x_out(:,1), x_out(:,2), 'b-', 'LineWidth',1); hold on;
title('Trajectory with AGV Visualization');
xlabel('X (m)'); ylabel('Y (m)'); grid on; axis equal;
step = round(length(t_out)/10);
for i = 1:step:length(t_out)
    drawAGV(x_out(i,1), x_out(i,2), x_out(i,3), params.width, params.L, 'r');
end
hold off;

%% -----------------------
%% 5. ROS2 Publication for Gazebo Integration
%% -----------------------
% Initialize ROS2 (MATLAB ROS Toolbox for ROS2 R2021b or later)
ros2("nodeName", "agv_node");

odomPub = ros2publisher("/odom", "nav_msgs/msg/Odometry");
odomMsg = ros2message(odomPub);

for i = 1:length(t_out)
    odomMsg.pose.pose.position.x = x_out(i,1);
    odomMsg.pose.pose.position.y = x_out(i,2);
    
    % Convert orientation (yaw only) to quaternion [w, x, y, z]
    quat = eul2quat([x_out(i,3), 0, 0]); 
    odomMsg.pose.pose.orientation.w = quat(1);
    odomMsg.pose.pose.orientation.x = quat(2);
    odomMsg.pose.pose.orientation.y = quat(3);
    odomMsg.pose.pose.orientation.z = quat(4);
    
    send(odomPub, odomMsg);
    pause(dt); % to mimic real-time publishing
end

%% -----------------------
%% 6. Functions
%% -----------------------

% AGV kinematic model function
function dxdt = agv_kinematic_model(t, x, u, params, t_sim)
    % State vector: x = [x, y, theta, v, delta]
    idx = max(1, min(length(t_sim), round((t - t_sim(1))/(t_sim(end)-t_sim(1))*(length(t_sim)-1)) + 1));
    a = u(1, idx);
    delta_dot = u(2, idx);
    
    dxdt = zeros(5,1);
    dxdt(1) = x(4)*cos(x(3));              % dx/dt
    dxdt(2) = x(4)*sin(x(3));              % dy/dt
    dxdt(3) = x(4)*tan(x(5))/params.L;       % dtheta/dt
    dxdt(4) = a;                         % dv/dt
    dxdt(5) = delta_dot;                 % ddelta/dt
end

% Function to draw the AGV as a polygon on the plot
function drawAGV(x, y, theta, width, length, color)
    % Define rectangle corners relative to vehicle CG
    corners = [ -length/2,  width/2;
                 length/2,  width/2;
                 length/2, -width/2;
                -length/2, -width/2];
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    transformed = (R * corners')';
    transformed(:,1) = transformed(:,1) + x;
    transformed(:,2) = transformed(:,2) + y;
    
    pgon = polyshape(transformed(:,1), transformed(:,2));
    plot(pgon, 'FaceColor', color, 'FaceAlpha', 0.3);
    
    % Draw an arrow indicating vehicle heading
    arrow_len = length * 0.8;
    quiver(x, y, arrow_len*cos(theta), arrow_len*sin(theta), 0, 'k', 'LineWidth',1.5);
end
