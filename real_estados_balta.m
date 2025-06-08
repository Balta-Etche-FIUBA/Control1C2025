% Your identified plant matrices (example values you gave)
A = [ -1.0135,  15.2396,       0,       0;
     -15.2396,  -1.0135, -0.3275,  0.9747;
           0,        0, -30.5063, 91.3426;
           0,        0, -91.3426, -30.5063 ];

B = [0; 0; 0; 131.4702];

C = [-6.2601, 94.1305, 0, 0];  % Your output: pendulum angle approx.

D = 0;

%% 1) Scale states to improve conditioning
% Compute state scaling factors (e.g. max abs row sums)
scale_factors = max(abs(A),[],2); 
scale_factors(scale_factors==0) = 1; % avoid division by zero

T = diag(scale_factors);          % scaling matrix
T_inv = inv(T);

% Scale A,B,C
A_scaled = T_inv * A * T;
B_scaled = T_inv * B;
C_scaled = C * T;

% Display condition number before and after scaling
cond_before = cond(A);
cond_after = cond(A_scaled);
fprintf('Condition number before scaling: %g\n', cond_before);
fprintf('Condition number after scaling: %g\n', cond_after);

%% 2) Controller pole placement (continuous-time)

% Choose desired poles (try stable but not too fast)
desired_poles_c = [-3+6i, -3-6i, -30+70i, -30-70i];

% Place poles with scaled system
K_scaled = acker(A_scaled, B_scaled, desired_poles_c);

% Convert K back to original coordinates
K = K_scaled * T_inv;

%% 3) Observer pole placement (continuous-time)

% Choose observer poles ~3-5 times faster than controller poles
observer_pole = -80;
observer_poles_c = observer_pole * ones(4,1);

% Place observer poles (on transposed system)
L_scaled = acker(A_scaled.', C_scaled.', observer_poles_c).';

% Convert L back to original coordinates
L = T * L_scaled;

%% 4) Build closed-loop matrices for simulation

Acl = A - B * K;
Aob = A - L * C;

%% 5) Display gains

disp('State feedback gain K:');
disp(K);

disp('Observer gain L:');
disp(L);

%% 6) Optional: simulate closed loop response

% Initial states (scaled)
x0 = [1/3.7; -1/6.6797; 1/1.236; 0];
x0_scaled = T * x0;

% Define simulation time
tspan = 0:0.01:5;

% Define system with observer error dynamics for simulation (optional)

% You can simulate your closed-loop system here, or export gains to Simulink

% Combined state vector: z = [x; x_hat]

n = size(A,1);

Acl = A - B*K;           % closed loop dynamics with state feedback
Aob = A - L*C;           % observer error dynamics

% Build augmented system dynamics for [x; x_hat]

% System equations:
% x_dot = A*x + B*u,  u = -K*x_hat
% x_hat_dot = A*x_hat + B*u + L*(y - C*x_hat)
%          = (A - B*K - L*C)*x_hat + L*C*x

A_aug = [A,            -B*K;
         L*C,   A - B*K - L*C];

B_aug = zeros(2*n,1); % no external input (assuming regulation problem)

C_aug = [C, zeros(size(C))];

D_aug = 0;

sys_cl = ss(A_aug, B_aug, C_aug, D_aug);

%% 5) Simulate closed loop response

tspan = 0:0.001:2;  % 2 seconds simulation

% Initial conditions:
% start with plant away from zero, observer at zero
x0 = [1/3.7; -1/6.6797; 1/1.236; 0]; % initial real plant states
xhat0 = zeros(n,1);                   % initial estimated states

z0 = [x0; xhat0];

% simulate zero input response (regulation to zero)
[y,t,x] = initial(sys_cl, z0, tspan);

%% 6) Plot results

figure;
subplot(2,1,1)
plot(t, x(:,1:n));
title('Actual plant states');
xlabel('Time (s)');
ylabel('States');
legend('x_1','x_2','x_3','x_4');

subplot(2,1,2)
plot(t, x(:,n+1:end));
title('Estimated states by observer');
xlabel('Time (s)');
ylabel('States');
legend('x_{hat1}','x_{hat2}','x_{hat3}','x_{hat4}');
