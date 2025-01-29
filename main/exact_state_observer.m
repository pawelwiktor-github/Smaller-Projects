%Exact State Observer (Integral) for n order system

%% Input parameter definition
clear; clc;

% Input parameters
n = input('Specify system row (n): '); % System order
A = input('Specify matrix A (in the form of an n x n matrix): '); % Matrix A
B = input('Give matrix B (in the form of an n x 1 matrix): '); % Matrix B
C = input('Give matrix C (in the form of a 1 x n matrix): '); % Matrix C
D = input('Give final matrix D: '); % Matrix D
T = input('Specify end time of observation window (T): '); % Length of window
beta = input('Give the matrix of beta weights (in the form of diagonal matrix beta*I n x n): '); % Beta weights
tau = input('Specify integration step: '); % Number of grid points to be integrated

% Checking the dimensions of the given matrices
if ~isequal(size(A), [n, n])
    error('Matrix A must have dimensions %d x %d.', n, n);
end
if ~isequal(size(B), [n, 1])
    error('Matrix B must have dimensions %d x 1.', n);
end
if ~isequal(size(C), [1, n])
    error('Matrix C must have dimensions 1 x %d.', n);
end
if length(beta) ~= n
    error('The vector of beta weights must have length %d.', n);
end

%% Simulation parameters
t = linspace(0, T, tau); % simulation time
n = size(A, 1); % state dimension
dt = t(2)-t(1); % time step
E = eye(n); % unit matrix for vectors e_i

%% Simulation start-up
T_final = 60; % final time of simulation (executed in model)
simOut = sim('exact_state_observer_model');

%% Calculation of M, G1, G2 
% Iterative integration
for i= 1:n
    W = [A, beta(i,i)*(B*B'); C'*C, -A'];
    M_i = zeros(n, n);
    for j = 1:length(t)
        t_idx = t(j);
        fi_t = expm(W*t(j));
        fi11_t = fi_t(1:n,1:n);
        fi21_t = fi_t(1+n:n+n,1:n);
        integrand = expm(-A'*(T-t_idx)) * C' * C * fi11_t;
        if j == 1 || j == length(t)
            M_i = M_i + T * integrand * dt; 
        else
            M_i = M_i + integrand * dt; 
        end
    end
    e_j = E(:, i);
    for k = 1:length(t)
        t_idx = t(k);
        fi_t = expm(W*t(k));
        fi11_t = fi_t(1:n,1:n);
        fi21_t = fi_t(1+n:n+n,1:n);
        p1_t =  fi11_t * (M_i \ e_j); 
        p2_t =  fi21_t * (M_i \ e_j); 
        P1_t = p1_t';
        P2_t = p2_t';
        G1_t(i, k) = P1_t*C';
        G2_t(i, k) = P2_t*B;
    end
end
J = 0;
g1_t = 0;
g2_t = 0;
[numRows,numCols] = size(G1_t);

for z = 1:numCols
    for m = 1:numRows
        g1_t = g1_t + (G1_t(m, z))^2;
        g2_t = g2_t + beta(m, m)*(G2_t(m, z))^2;
    end

    t_idx = t(z);
    integrand = g1_t + g2_t;
    if z == 1 || z == length(t)
        J = J + T * integrand * dt; 
    else
        J = J + integrand * dt; 
    end    

    g1_t = 0;
    g2_t = 0;
end
J = sqrt(J); % observer norm

%% Results visualization
% Iterative G courses (based on system order)
figure;
for i = 1:n
    subplot(n, 2, 2*i-1);
    plot(t, G1_t(i, :));
    title(['$G_{1' num2str(i) '}(t)$'], 'Interpreter', 'latex', 'FontSize', 16);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 16);
    ylabel('Values', 'Interpreter', 'latex', 'FontSize', 16);
    legend(['$G_{1' num2str(i) '}$'], 'Interpreter', 'latex', 'FontSize', 14);
    grid on;
    ax = gca; 
    ax.FontSize = 12;

    subplot(n, 2, 2*i);
    plot(t, G2_t(i, :));
    title(['$G_{2' num2str(i) '}(t)$'], 'Interpreter', 'latex', 'FontSize', 16);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 16);
    ylabel('Values', 'Interpreter', 'latex', 'FontSize', 16);
    legend(['$G_{2' num2str(i) '}$'], 'Interpreter', 'latex', 'FontSize', 14);
    grid on;
    ax = gca; 
    ax.FontSize = 12; 
end

%% State estimation 
% on-line window (T)
for i=1:(length(y)-tau)
    Y_comp = 0;
    U_comp = 0;
    for l=1+(i-1):tau+(i-1)
        if y(l) == y(end) || u(l) == u(end)
            break
        end
        y_comp = G1_t(:, l-(i-1)) * y(l); 
        u_comp = G2_t(:, l-(i-1)) * u(l);
        integrand_y = y_comp;
        Y_comp = Y_comp + integrand_y * dt;    
        integrand_u = u_comp;
        U_comp = U_comp + integrand_u * dt;   
    end
    x_t(:,i) = Y_comp + U_comp;
    if mod(i, 5000) == 0
        fprintf('Iteration %d completed. There are $d left.\n', i, (length(y)-tau)-i);
    end
end

%% Control input signal and output signal

t_plot = linspace(0, T_final, length(u));

figure;
subplot(2,1,1)
plot(t_plot, u);
title(['Input signal graph'], 'Interpreter', 'latex', 'FontSize', 16);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('Value', 'Interpreter', 'latex', 'FontSize', 16);
grid on;
legend('Control', 'Interpreter', 'latex', 'FontSize', 14);
subplot(2,1,2)
plot(t_plot, y);
title(['Output signal graph'], 'Interpreter', 'latex', 'FontSize', 16);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('Value', 'Interpreter', 'latex', 'FontSize', 16);
grid on;
legend('Output', 'Interpreter', 'latex', 'FontSize', 14);
ax = gca; 
ax.FontSize = 12; 

%% Estimate plot

figure;
plot(x_t(1,:));
hold on;
plot(x_t(2,:));
title(['State course'], 'Interpreter', 'latex', 'FontSize', 16);
xlabel('Sample', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('Value', 'Interpreter', 'latex', 'FontSize', 16);
grid on;
legend(['$x_{1}$'], ['$x_{2}$'], 'Interpreter', 'latex', 'FontSize', 14);
ax = gca; 
ax.FontSize = 12; 

%% Validation (quality indicators)

u_calc = u(tau+1:end);
y_calc = C * x_t + D * u_calc';
y_calc = y_calc';
N = length(y_calc);
y_ref = y(tau+1:end);
diff = y_ref - y_calc;
sigma = (1/N)*(sum(abs(diff).^2));
sigma = sqrt(sigma);
fprintf('Standard deviation: %.15f\n', sigma);

figure;
plot(y_ref(:), "*", MarkerSize=5);
hold on;
plot(y_calc(:), LineWidth=3);
title('Output signal comparison', 'Interpreter', 'latex', 'FontSize', 16);
xlabel('Sample', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('Value', 'Interpreter', 'latex', 'FontSize', 14);
grid on;
legend(['Reference signal'], ['Signal based on state estimation'], 'Interpreter', 'latex');
ax = gca; 
ax.FontSize = 12; 

%% Observer's norm graph
T_change = 0.2:0.2:5; % changing length of window

for zz = 1:length(T_change)
T = T_change(zz);
t = linspace(0, T, tau);  % simulation time
dt = t(2)-t(1); % time step

for i = 1:n
    W = [A, beta(i,i)*(B*B'); C'*C, -A'];
    M_i = zeros(n, n);
    for j = 1:length(t)
        t_idx = t(j);
        fi_t = expm(W*t(j));
        fi11_t = fi_t(1:n,1:n);
        fi21_t = fi_t(1+n:n+n,1:n);
        integrand = expm(-A'*(T-t_idx)) * C' * C * fi11_t;
        if j == 1 || j == length(t)
            M_i = M_i + T * integrand * dt; 
        else
            M_i = M_i + integrand * dt; 
        end
    end
    e_j = E(:, i);
    for k = 1:length(t)
        t_idx = t(k);
        fi_t = expm(W*t(k));
        fi11_t = fi_t(1:n,1:n);
        fi21_t = fi_t(1+n:n+n,1:n);
        p1_t =  fi11_t * (M_i \ e_j); 
        p2_t =  fi21_t * (M_i \ e_j); 
        P1_t = p1_t';
        P2_t = p2_t';
        G1_t(i, k) = P1_t*C';
        G2_t(i, k) = P2_t*B;
    end
end
J = 0;
g1_t = 0;
g2_t = 0;
[numRows,numCols] = size(G1_t);

for z = 1:numCols
    for m = 1:numRows
        g1_t = g1_t + (G1_t(m, z))^2;
        g2_t = g2_t + beta(m, m)*(G2_t(m, z))^2;
    end

    t_idx = t(z);
    integrand = g1_t + g2_t;
    if z == 1 || z == length(t)
        J = J + T * integrand * dt; 
    else
        J = J + integrand * dt; 
    end    

    g1_t = 0;
    g2_t = 0;
end
J = sqrt(J);
J_zmiana(zz) = J;
end

figure;
plot(T_change, J_zmiana);
title('Graph of observer norm for increasing T', 'Interpreter', 'latex', 'FontSize', 16);
xlabel('T [s]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel('J', 'Interpreter', 'latex', 'FontSize', 14);
axis([0.2, 5, 0, 20]);
grid on;
legend(['Observer norm for 2nd order system'], 'Interpreter', 'latex');
ax = gca; 
ax.FontSize = 12; 