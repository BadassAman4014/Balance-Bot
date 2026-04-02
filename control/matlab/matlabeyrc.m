% L12_2dfan.m - ducted fan example for L12.2
% RMM, 14 Jan 03
%%
%% Ducted fan dynamics
%%
% These are the dynamics for the ducted fan, written in state space form.

% System parameters
J = 0.0475;    % Inertia around pitch axis
m = 1.5;       % Mass of fan
r = 0.25;      % Distance to flaps
g = 10;        % Gravitational constant
gamma = 0.51;  % Counterweighted mass
d = 0.2;       % Damping factor (estimated)
l = 0.05;      % Offset of center of mass

% System matrices (entire plant: 2 input, 2 output)
A = [ 0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1;
      0 0 -gamma -d/m 0 0;
      0 0 0 0 -d/m 0;
      0 0 -m*g*l/J 0 0 0 ];

B = [ 0 0;
      0 0;
      0 0;
      1/m 0;
      0 1/m;
      r/J 0 ];

C = [ 1 0 0 0 0 0;
      0 1 0 0 0 0 ];

D = [ 0 0;
      0 0 ];

%%
%% Construct inputs and outputs corresponding to steps in xy position
%%
% The vectors xd and yd correspond to the states that are the desired
% equilibrium states for the system. The matrices Cx and Cy are the
% corresponding outputs.

% Desired equilibrium states and outputs
xd = [1; 0; 0; 0; 0; 0];
Cx = [1 0 0 0 0 0];
yd = [0; 1; 0; 0; 0; 0];
Cy = [0 1 0 0 0 0];

% Closed loop system dynamics
% xdot = Ax + B u => xdot = (A-BK)x + K xd
% u = -K(x - xd) y = Cx
% The closed loop dynamics can be simulated using the "step" command,
% with K*xd as the input vector (assumes that the "input" is unit size,
% so that xd corresponds to the desired steady state).

%%
%% LQR design
%%

% Start with a diagonal weighting
Q1 = diag([1, 1, 1, 1, 1, 1]);
R1a = 0.1 * diag([1, 1]);
K1a = lqr(A, B, Q1, R1a);

% Close the loop: xdot = Ax + B K (x-xd)
H1ax = ss(A - B * K1a, B(:,1) * K1a(1,:) * xd, Cx, 0);
H1ay = ss(A - B * K1a, B(:,2) * K1a(2,:) * yd, Cy, 0);
figure(1);
step(H1ax, H1ay, 10);
legend('x', 'y');

% Look at different input weightings
R1b = diag([1, 1]);
K1b = lqr(A, B, Q1, R1b);
H1bx = ss(A - B * K1b, B(:,1) * K1b(1,:) * xd, Cx, 0);

R1c = diag([10, 10]);
K1c = lqr(A, B, Q1, R1c);
H1cx = ss(A - B * K1c, B(:,1) * K1c(1,:) * xd, Cx, 0);

figure(2);
step(H1ax, H1bx, H1cx, 10);
legend('rho = 0.1', 'rho = 1', 'rho = 10');

% Output weighting
Q2 = [Cx; Cy]' * [Cx; Cy];
R2 = 0.1 * diag([1, 1]);
K2 = lqr(A, B, Q2, R2);

H2x = ss(A - B * K2, B(:,1) * K2(1,:) * xd, Cx, 0);
H2y = ss(A - B * K2, B(:,2) * K2(2,:) * yd, Cy, 0);

figure(3);
step(H2x, H2y, 10);
legend('x', 'y');

%%
%% Physically motivated weighting
%%

% Shoot for 1 cm error in x, 10 cm error in y. Try to keep the angle
% less than 5 degrees in making the adjustments. Penalize side forces
% due to loss in efficiency.

Q3 = diag([100, 10, 2*pi/5, 0, 0, 0]);
R3 = 0.1 * diag([1, 10]);
K3 = lqr(A, B, Q3, R3);

H3x = ss(A - B * K3, B(:,1) * K3(1,:) * xd, Cx, 0);
H3y = ss(A - B * K3, B(:,2) * K3(2,:) * yd, Cy, 0);

figure(4);
step(H3x, H3y, 10);
legend('x', 'y');

%%
%% Velocity control
%%

% In this example, we modify the system so that we control the velocity of
% the system in the x direction. We ignore the dynamics in the vertical (y)
% direction. These dynamics demonstrate the role of the feedforward system
% since the equilibrium point corresponding to vd neq 0 requires a nonzero input.

% For this example, we use a control law u = -K(x-xd) + ud and convert
% this to the form u = -K x + N r, where r is the reference input and
% N is computed as described in class.

% Extract system dynamics: theta, xdot, thdot
Av = A([3 4 6], [3 4 6]);
Bv = B([3 4 6], 1);
Cv = [0 1 0]; % Choose vx as output
Dv = 0;

% Design the feedback term using LQR
Qv = diag([2*pi/5, 10, 0]);
Rv = 0.1;
Kv = lqr(Av, Bv, Qv, Rv);

% Design the feedforward term by solving for eq pt in terms of reference r
T = [Av Bv; Cv Dv]; % System matrix
Nxu = T \ [0; 0; 0; 1]; % Compute [Nx; Nu]
Nx = Nxu(1:3); % Extract Nx
Nu = Nxu(4); % Extract Nu
N = Nu + Kv * Nx; % Compute feedforward term

%%
%% Design #1: no feedforward input, ud
%%
Nv1 = [0; 1; 0];
Hv1 = ss(Av - Bv * Kv, Bv * Kv * Nx, Cv, 0);
figure(5);
step(Hv1, 10);

%%
%% Design #2: compute feedforward gain corresponding to equilibrium point
%%
Hv2 = ss(Av - Bv * Kv, Bv * N, Cv, 0);
figure(6);
step(Hv2, 10);

%%
%% Design #3: integral action
%%

% Add a new state to the system that is given by xidot = v - vd. We
% construct the control law by computing an LQR gain for the augmented system.

Ai = [Av, [0; 0; 0]; [Cv, 0]];
Bi = [Bv; 0];
Ci = [Cv, 0];
Di = Dv;

% Design the feedback term, including weight on integrator error
Qi = diag([2*pi/5, 10, 0, 10]);
Ri = 0.1;
Ki = lqr(Ai, Bi, Qi, Ri);

% Desired state (augmented)
xid = [0; 1; 0; 0];

% Construct the closed loop system (including integrator)
Hi = ss(Ai - Bi * Ki, Bi * Ki * xid - [0; 0; 0; Ci * xid], Ci, 0);
figure(7);
step(Hi, 10);

% Inverted Pendulum Parameters
m = 0.5;       % Mass of pendulum (kg)
M = 0.5;       % Mass of cart (kg)
l = 0.3;       % Length of pendulum (m)
g = -9.81;      % Acceleration due to gravity (m/s^2)
b = 0.1;       % Damping coefficient (N*m*s/rad)

% System Matrices
A = [0 1 0 0;
     0 -b/M -m*g/M 0;
     0 0 0 1;
     0 -b/(M*l) -(m+M)*g/(M*l) 0];
 
B = [0; 1/M; 0; 1/(M*l)];
 
C = [1 0 0 0;
     0 0 1 0];
 
D = [0; 0];

% Define desired equilibrium states and outputs
xd = [0; 0; pi; 0];  % Desired state (e.g., cart at origin and pendulum upright)
Cx = [1 0 0 0];
Cy = [0 1 0 0];

% LQR Design
Q = diag([10, 1, 100, 1]);  % State weighting matrix
R = 1;                     % Input weighting matrix

% Compute LQR gain
K = lqr(A, B, Q, R);

% Define closed-loop system
Acl = A - B*K;
Bcl = B;
Ccl = C;
Dcl = D;

% Simulation parameters
t = 0:0.01:10; % Time vector
u = zeros(size(t)); % No reference input

% Initial condition
x0 = [0; 0; pi - 0.1; 0]; % Small perturbation from upright position

% Create state-space model for the closed-loop system
sys_cl = ss(Acl, Bcl, Ccl, Dcl);

% Simulate the system
[y, t, x] = lsim(sys_cl, u, t, x0);

% Plot the results
figure;
hold on;
grid on;

% Parameters for plotting
cartWidth = 0.4; % Width of the cart
cartHeight = 0.2; % Height of the cart
pendulumColor = 'r'; % Pendulum color
cartColor = 'b'; % Cart color

% Loop for animation
for k = 1:length(t)
    % Extract cart and pendulum states
    cartPosition = y(k,1);
    theta = y(k,2); % Pendulum angle
    pendulumLength = l;
    
    % Cart coordinates
    cartX = [-cartWidth/2, -cartWidth/2, cartWidth/2, cartWidth/2];
    cartY = [0, cartHeight, cartHeight, 0];
    
    % Pendulum coordinates
    pendulumX = [0, pendulumLength*sin(theta) + cartPosition];
    pendulumY = [0, -pendulumLength*cos(theta)];
    
    % Plot the cart
    fill(cartX + cartPosition, cartY, cartColor, 'EdgeColor', 'k', 'LineWidth', 2);
    
    % Plot the pendulum
    plot(pendulumX, pendulumY, 'LineWidth', 2, 'Color', pendulumColor);
    
    % Axis settings
    axis([-2 2 -0.5 1]);
    xlabel('Position (m)');
    ylabel('Height (m)');
    title('Inverted Pendulum Balancing');
    drawnow;
    
    % Clear plot for next frame
    clf;
end

