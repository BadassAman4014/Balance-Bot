function dx = pendcart(x, m, M, L, g, d, u)
    % Extracting the state variables for clarity
    Sx = sin(x(3));   % sin(theta)
    Cx = cos(x(3));   % cos(theta)
    
    % Denominator D
    D = m * L^2 * (M + m * (1 - Cx^2));
    
    % State derivatives
    dx(1, 1) = x(2);
    dx(2, 1) = (1/D) * (-m^2 * L^2 * g * Cx * Sx + m * L^2 * (m * L * x(4)^2 * Sx - d * x(2))) ...
                + m * L^2 * (1/D) * u;
    dx(3, 1) = x(4);
    dx(4, 1) = (1/D) * ((m + M) * m * g * L * Sx - m * L * Cx * (m * L * x(4)^2 * Sx - d * x(2))) ...
                - m * L * Cx * (1/D) * u;
end

%%
clear all; %#ok<CLALL>
close all;

% Parameters
m = 0.5;    % Mass of the pendulum (kg)
M = 0.248;    % Mass of the cart (kg)
L = 0.077;    % Length of the pendulum (m)
g = -9.8;  % Gravitational acceleration (m/s^2), negative due to inverted pendulum
d = 0.5;    % Damping coefficient (N/m/s)
b = 1;    % Pendulum up (b=1 for upward configuration)

% State-Space Matrices
A = [0  1   0                   0;
     0 -d/M b*m*g/M             0;
     0  0   0                   1;
     0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
 
B = [0; 
     1/M; 
     0; 
     b*1/(M*L)];


%% Design LQR controller
Q = [200 0 0 0;    % Penalize cart position
     0 500 0 0;     % Cart velocity
     0 0 1000 0;   % Penalize pendulum angle (theta)
     0 0 0 100];   % Pendulum angular velocity

R = 0.01;  % Scalar (input cost)

%% Simulate closed-loop system
tspan = 0:0.01:8;            % Time span for the simulation
x0 = [0; 0; pi+0.1; 0];      % Initial condition (initial state)
wr = [0.5; 0; pi; 0];           % Reference position (desired state)
K = lqr(A, B, Q, R);          % LQR gain matrix
K
% Define the control law within the pendulum dynamics function
u = @(x) -K*(x - wr);         % Control law

% Define a new function to include the control law
pendcart_with_control = @(t,x) pendcart(x, m, M, L, g, d, u(x));

% Simulate using ode45
[t, x] = ode45(pendcart_with_control, tspan, x0);

%% Animate the results
% Parameters for the cart and pendulum visualization
cart_width = 0.086;
cart_height = 0.016;
wheel_radius = 0.04;
pendulum_length = L;  % Length of the pendulum

% Create the figure for the animation
figure;
axis equal;
hold on;

% Set proper axis limits based on the cart's and pendulum's motion
xlim([-1 3]);  % Adjust this range based on expected cart movement
ylim([-pendulum_length-0.1, pendulum_length+1]);  % Adjust to fit pendulum swing

% Plot cart as a rectangle
cart = rectangle('Position', [x(1,1)-cart_width/2, -cart_height/2, cart_width, cart_height], 'Curvature', 0.1, 'FaceColor', [0 0.5 1]);

% Plot pendulum as a line
pendulum = plot([x(1,1), x(1,1) + pendulum_length * sin(x(1,3))], [0, -pendulum_length * cos(x(1,3))], 'LineWidth', 2);

% Plot the wheels of the cart
wheel1 = rectangle('Position', [x(1,1)-cart_width/2, -cart_height/2 - wheel_radius, 2*wheel_radius, wheel_radius], 'Curvature', [1,1], 'FaceColor', [0 0 0]);
wheel2 = rectangle('Position', [x(1,1)+cart_width/2-2*wheel_radius, -cart_height/2 - wheel_radius, 2*wheel_radius, wheel_radius], 'Curvature', [1,1], 'FaceColor', [0 0 0]);

% Update the animation for each time step
for i = 1:length(t)
    % Update the cart's position
    set(cart, 'Position', [x(i,1)-cart_width/2, -cart_height/2, cart_width, cart_height]);
    
    % Update the pendulum's position
    set(pendulum, 'XData', [x(i,1), x(i,1) + pendulum_length * sin(x(i,3))]);
    set(pendulum, 'YData', [0, -pendulum_length * cos(x(i,3))]);
    
    % Update the wheels' position
    set(wheel1, 'Position', [x(i,1)-cart_width/2, -cart_height/2 - wheel_radius, 2*wheel_radius, wheel_radius]);
    set(wheel2, 'Position', [x(i,1)+cart_width/2-2*wheel_radius, -cart_height/2 - wheel_radius, 2*wheel_radius, wheel_radius]);
    
    % Pause to create animation effect
    pause(0.01);
end
close all;