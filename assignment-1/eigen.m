% Define the system matrices for the equation: d²x/dt² + 3(dx/dt) + 5x(t) = 3u(t)
A = [0 1; -5 -3];  % State matrix
B = [0; 3];        % Input matrix
C = [1 0];         % Output matrix (assuming we're measuring position)
D = 0;             % Direct transmission term

% Display the state space model
disp('State Space Model:')
disp('A = ')
disp(A)
disp('B = ')
disp(B)

% Calculate eigenvalues and eigenvectors
[V, D] = eig(A);
eigenvalues = diag(D);

% Display eigenvalues
disp('Eigenvalues:')
disp(eigenvalues)

% Display eigenvectors
disp('Eigenvectors:')
disp(V)

% Create state space model
sys = ss(A, B, C, D);

% Display the transfer function
tf_sys = tf(sys);
disp('Transfer Function:')
disp(tf_sys)

% You can also verify the characteristic equation manually
disp('Characteristic Equation:')
char_eq = poly(A);
disp(['λ² + ' num2str(char_eq(2)) 'λ + ' num2str(char_eq(3)) ' = 0'])

% Optional: Plot step response to visualize system behavior
figure;
step(sys);
title('Step Response of the System');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;
