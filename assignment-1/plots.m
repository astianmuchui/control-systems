% Control III - Assignment 1
% Sebastian Muchui SCP222-1166/2022
% Analysis and Design of State Space Controller for G(s) = 10/((s+1)(s+2)(s+3))

close all;
clear all;
clc;

%% a) i) State Space Model
% Define the system in transfer function form
num = 10;
den = [1 6 11 6]; % s³ + 6s² + 11s + 6

% Convert to state space (controllable canonical form)
[A, B, C, D] = tf2ss(num, den);

% Display the state space matrices
disp('State Space Model in Controllable Canonical Form:');
disp('A =');
disp(A);
disp('B =');
disp(B);
disp('C =');
disp(C);
disp('D =');
disp(D);

%% a) iii) Verify the transfer function from the state space model
[num_ss, den_ss] = ss2tf(A, B, C, D);
sys_ss = tf(num_ss, den_ss);

disp('Transfer Function from State Space Model:');
disp(sys_ss);

% Original transfer function for comparison
sys_orig = tf(num, den);
disp('Original Transfer Function:');
disp(sys_orig);

%% a) iv) Eigenvalues, Eigenvectors, Modal Matrix and Inverse Modal Matrix
% Calculate eigenvalues
eigenvalues = eig(A);
disp('Eigenvalues:');
disp(eigenvalues);

% Calculate eigenvectors and modal matrix
[M, D] = eig(A);
disp('Modal Matrix (Eigenvectors):');
disp(M);
disp('Diagonal Matrix of Eigenvalues:');
disp(D);

% Calculate inverse modal matrix
M_inv = inv(M);
disp('Inverse Modal Matrix:');
disp(M_inv);

%% a) v) Transform the system using the modal matrix
% Transformed system matrices
A_transformed = M_inv * A * M;
B_transformed = M_inv * B;
C_transformed = C * M;
D_transformed = D;

disp('Transformed System Matrices:');
disp('A_transformed =');
disp(A_transformed);
disp('B_transformed =');
disp(B_transformed);
disp('C_transformed =');
disp(C_transformed);
disp('D_transformed =');
disp(D_transformed);

% Verify eigenvalues of transformed A matrix
eig_transformed = eig(A_transformed);
disp('Eigenvalues of Transformed System:');
disp(eig_transformed);

%% a) vi) Transition Matrix and System Solution
% Define time vector for simulation
t = 0:0.01:5;

% Calculate transition matrix symbolically
syms tau;
phi_sym = expm(A*tau);
phi = matlabFunction(phi_sym);

% Initialize unit step input
u = ones(size(t));

% Initialize state vector
x0 = zeros(3,1);

% Calculate system response using state transition approach
n = length(t);
x = zeros(3, n);
y = zeros(1, n);

for i = 1:n
    % Calculate state at time t(i)
    x(:,i) = phi(t(i)) * x0;
    
    % Add the effect of input
    for j = 1:i-1
        x(:,i) = x(:,i) + phi(t(i)-t(j)) * B * u(j) * 0.01; % 0.01 is dt
    end
    
    % Calculate output
    y(i) = C * x(:,i) + D * u(i);
end

% Plot the system response
figure;
plot(t, y);
title('System Response to Unit Step Input (Transition Matrix Approach)');
xlabel('Time (s)');
ylabel('Output');
grid on;

% Alternative approach using lsim
sys_ss_model = ss(A, B, C, D);
[y_lsim, t_lsim, x_lsim] = lsim(sys_ss_model, u, t, x0);

figure;
plot(t_lsim, y_lsim);
title('System Response to Unit Step Input (Using lsim)');
xlabel('Time (s)');
ylabel('Output');
grid on;

%% b) State Space Controller Design

% Design 1: Place poles at s = -4, -6, -7
desired_poles_1 = [-4; -6; -7];
K1 = acker(A, B, desired_poles_1);
disp('Controller Gain K1 (Poles at -4, -6, -7):');
disp(K1);

% Design 2: Place poles at s = -3, -4+j5, -4-j5, -6
desired_poles_2 = [-3; -4+5j; -4-5j; -6];
% Since our system is 3rd order, we need to drop one pole
desired_poles_2 = desired_poles_2(1:3);
K2 = acker(A, B, desired_poles_2);
disp('Controller Gain K2 (Poles at -3, -4+j5, -4-j5):');
disp(K2);

%% Simulation and Performance Analysis of Controlled Systems

% Closed-loop systems
A_cl1 = A - B*K1;
sys_cl1 = ss(A_cl1, B, C, D);

A_cl2 = A - B*K2;
sys_cl2 = ss(A_cl2, B, C, D);

% Simulate step response for both controllers
[y1, t1] = step(sys_cl1, t);
[y2, t2] = step(sys_cl2, t);

% Plot step responses
figure;
plot(t1, y1, 'b-', t2, y2, 'r--', 'LineWidth', 1.5);
title('Step Response Comparison of Two Controllers');
xlabel('Time (s)');
ylabel('Output');
legend('Controller 1 (Poles at -4, -6, -7)', 'Controller 2 (Poles at -3, -4±j5)');
grid on;

% Calculate performance metrics
% For Controller 1
S1 = stepinfo(y1, t1);
disp('Performance Metrics for Controller 1:');
disp(S1);

% For Controller 2
S2 = stepinfo(y2, t2);
disp('Performance Metrics for Controller 2:');
disp(S2);

% Plot control inputs
u1 = -K1 * x_lsim';
u2 = -K2 * x_lsim';

figure;
plot(t, u1, 'b-', t, u2, 'r--', 'LineWidth', 1.5);
title('Control Input Comparison');
xlabel('Time (s)');
ylabel('Control Input u(t)');
legend('Controller 1', 'Controller 2');
grid on;

%% Discussion of Performance Parameters

% Calculate settling time, rise time, overshoot, and steady-state values
fprintf('\nPerformance Comparison:\n');
fprintf('Controller 1 (Poles at -4, -6, -7):\n');
fprintf('Rise Time: %.4f s\n', S1.RiseTime);
fprintf('Settling Time: %.4f s\n', S1.SettlingTime);
fprintf('Overshoot: %.2f%%\n', S1.Overshoot);
fprintf('Peak: %.4f\n', S1.Peak);
fprintf('Peak Time: %.4f s\n', S1.PeakTime);
fprintf('Steady-State Value: %.4f\n', y1(end));

fprintf('\nController 2 (Poles at -3, -4±j5):\n');
fprintf('Rise Time: %.4f s\n', S2.RiseTime);
fprintf('Settling Time: %.4f s\n', S2.SettlingTime);
fprintf('Overshoot: %.2f%%\n', S2.Overshoot);
fprintf('Peak: %.4f\n', S2.Peak);
fprintf('Peak Time: %.4f s\n', S2.PeakTime);
fprintf('Steady-State Value: %.4f\n', y2(end));

% Calculate pole positions for both designs
eig_cl1 = eig(A_cl1);
eig_cl2 = eig(A_cl2);

fprintf('\nClosed-Loop Poles:\n');
fprintf('Controller 1: \n');
disp(eig_cl1);
fprintf('Controller 2: \n');
disp(eig_cl2);

% Plot pole-zero map for both controllers
figure;
pzmap(sys_cl1, 'b', sys_cl2, 'r');
title('Pole-Zero Map for Both Controllers');
legend('Controller 1', 'Controller 2');
grid on;
