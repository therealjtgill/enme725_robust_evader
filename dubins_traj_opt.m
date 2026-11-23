%% Unpack named states from state vector
function [pos_x, pos_y, theta] = unpack_state(x)
    pos_x = x(1);
    pos_y = x(2);
    theta = x(3);
end

%% Pack named states into one state vector
function x = pack_states(pos_x, pos_y, theta)
    x = zeros(3, 1);
    x(1) = pos_x;
    x(2) = pos_y;
    x(3) = theta;
end

%% Unpack state and input from decision vector
function [X, U] = unpack(z, nx, nu)
    % Horizon length
    N = (length(z) - nx) / (nx + nu); % Exclude final state
    X = reshape(z(1: nx*(N + 1)), nx, N + 1);
    U = reshape(z(nx * (N + 1) + 1:end), nu, N);
end

function x_dot = dubins_dynamics(x, u)
    [pos_x, pos_y, theta] = unpack_state(x);
    x_dot = pack_states(u(1) * cos(theta), u(1) * sin(theta), u(2));
end

%% Minimum effort cost function
function J = min_effort_cost(z, nx, nu, T)
    % z:    decision variables
    % nx:   size of state variables
    % nu:   size of input variable
    % T:    final time
    
    [~, U] = unpack(z, nx, nu);
    U_flat = reshape(U, 1, []);
    N = length(U);
    dt = T / N;
    
    J = dt * sum(U_flat.^2);
end


%% Region avoidance cost function
function J = avoidance_cost(z, nx, nu, T, R_avoid, c_avoid)
    [X, U] = unpack(z, nx, nu);
    U_flat = reshape(U, 1, []);
    N = length(U);
    dt = T / N;

    avoid_cost = 0;
    % avoidance cost per collocation point is a gaussian centered on
    % c_avoid, with a stddev of R_avoid. the idea is to allow the penalty
    % to taper off for large distances
    for k = 1:N
        avoid_cost = avoid_cost + (1/sqrt(2*pi* R_avoid)*exp(- dot(X(1:2,k) - c_avoid, X(1:2,k) - c_avoid)/(2*R_avoid^2)));
    end
    effort_cost = dt * sum(U_flat.^2);
    J = avoid_cost + effort_cost;
end

%% Region avoidance and dynamics constraints
% fmincon treats the nonlinear constraints as
%   cineq <= 0 (inequality constraints)
%   ceq = 0    (equality constraints)
function [cineq, ceq] = dynamics_avoid_constraints(z, nx, nu, T, x0, xT, R_avoid, c_avoid)
    % z:    decision variables
    % nx:   size of state variables
    % nu:   size of input variable
    % T:    final time
    % x0:   initial state
    % xT:   desired final state
    % R_avoid: radius of avoidance of stationary point
    % c_avoid: position of stationary point to avoid (2x1)
    
    [X, U] = unpack(z, nx, nu);
    
    N = length(U);
    dt = T/N;
    
    ceq = [];
    ceq = [ceq; X(:, 1) - x0; X(1:2, end) - xT(1:2)];
    % last_state_dynamics = dubins_dynamics(X(:, end - 1), U(:, end));
    % ceq = [ceq; last_state_dynamics(1:2)];

    for k = 1:N
        k1 = dubins_dynamics(X(:, k), U(:, k));
        k2 = dubins_dynamics(X(:, k) + k1 * dt / 2, U(:, k));
        k3 = dubins_dynamics(X(:, k) + k2 * dt / 2, U(:, k));
        k4 = dubins_dynamics(X(:, k) + k3 * dt, U(:, k));
    
        ceq = [ceq; X(:, k + 1) - X(:, k) - (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)];
    end
    
    % speed commands between [0,3] and angular speed commands between
    % [-2pi/3, 2pi/3] (swag)
    % cineq = [-U + [0, -2*pi/3]'; U - [18, 2*pi/3]'];
    % stack control inequalities vertically, makes it easier to add more
    % inequality constraints
    cineq = [];
    for k = 1:N
        cineq = [cineq; -U(1:2,k) + [0, -2*pi/3]'; U(1:2,k) - [18, 2*pi/3]'];
    end
    % avoid the stationary rock obstacle
    % R_avoid = 0.25;
    % c_avoid = (x0(1:2) + xT(1:2))/2;
    for k = 1:N
        % size(R_avoid^2 - dot(X(1:2,k) - c_avoid, X(1:2,k) - c_avoid))
        cineq = [cineq; R_avoid^2 - dot(X(1:2,k) - c_avoid, X(1:2,k) - c_avoid)];
    end
    
end

%% Dynamics constraints
% fmincon treats the nonlinear constraints as
%   cineq <= 0 (inequality constraints)
%   ceq = 0    (equality constraints)
function [cineq, ceq] = dynamics_constraints(z, nx, nu, T, x0, xT, R_avoid, c_avoid)
% z:    decision variables
% nx:   size of state variables
% nu:   size of input variable
% T:    final time
% x0:   initial state
% xT:   desired final state
% R_avoid: radius of avoidance of stationary point
% c_avoid: position of stationary point to avoid (2x1)

[X, U] = unpack(z, nx, nu);

N = length(U);
dt = T/N;

ceq = [];
ceq = [ceq; X(:, 1) - x0; X(1:2, end) - xT(1:2)];
% last_state_dynamics = dubins_dynamics(X(:, end - 1), U(:, end));
% ceq = [ceq; last_state_dynamics(1:2)];

for k = 1:N
    k1 = dubins_dynamics(X(:, k), U(:, k));
    k2 = dubins_dynamics(X(:, k) + k1 * dt / 2, U(:, k));
    k3 = dubins_dynamics(X(:, k) + k2 * dt / 2, U(:, k));
    k4 = dubins_dynamics(X(:, k) + k3 * dt, U(:, k));

    ceq = [ceq; X(:, k + 1) - X(:, k) - (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)];
end

% speed commands between [0,3] and angular speed commands between
% [-2pi/3, 2pi/3] (swag)
% cineq = [-U + [0, -2*pi/3]'; U - [18, 2*pi/3]'];
% stack control inequalities vertically, makes it easier to add more
% inequality constraints
cineq = [];
for k = 1:N
    cineq = [cineq; -U(1:2,k) + [0, -2*pi/3]'; U(1:2,k) - [18, 2*pi/3]'];
end

end

%% Optimization parameters
N = 100;
T = 4.5;
dt = T / N;

nx = 3;
nu = 2;

% Boundary conditions
x0 = zeros(nx, 1);
xT = zeros(nx, 1);
x0(3) = 0;
xT(1) = -2;
xT(2) = 2;
xT(3) = -pi/2;

R_avoid = 0.75;
c_avoid = (x0(1:2) + xT(1:2))/2;
% c_avoid = [-1.5;1];

z0 = zeros(nx * (N + 1) + nu * N, 1);

% Initial guess: linear interpolation
for k = 0:N
    z0(nx*k + 1: nx*(k + 1)) = x0 + (k/N)*(xT-x0);
end

% Set control input to half throttle for all motors
z0(nx*(k + 1) + 1:end) = 1;

%% Solve NLP

% Optimizer settings
options = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','iter', ...
    'MaxFunctionEvaluations',1e5);

tic

% [z, cost_val] = fmincon(@(z) min_effort_cost(z, nx, nu, T), z0, [], [], [], [], [], [], @(z) dynamics_avoid_constraints(z, nx, nu, T, x0, xT, R_avoid, c_avoid), options);

[z_penalty, ~] = fmincon(@(z) avoidance_cost(z, nx, nu, T, R_avoid, c_avoid), z0, [], [], [], [], [], [], @(z) dynamics_constraints(z, nx, nu, T, x0, xT, R_avoid, c_avoid), options);

[z, cost_val] = fmincon(@(z) min_effort_cost(z, nx, nu, T), z_penalty, [], [], [], [], [], [], @(z) dynamics_avoid_constraints(z, nx, nu, T, x0, xT, R_avoid, c_avoid), options);

t = toc;

% Extract state and input trajectories
[X,U] = unpack(z,nx,nu);

%% Make plots
plot_dubins_results(X, U, T, R_avoid, c_avoid);

title(['Collocation, cost = ' num2str(cost_val) ', time = ' num2str(t) '[s]'],FontSize=18)
