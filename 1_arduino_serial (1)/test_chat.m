%% Ajuste por mínimos cuadrados de un modelo diferencial de 3er orden
% y obtención de la función de transferencia G(s)

clear all;
clc;
data = load('datos_hoy.mat');
t      = data.mediciones.time;       % vector de tiempo
tita0  = data.mediciones.data(:,1);  % θ(t)
u0     = data.mediciones.data(:,3);  % u(t)

% 1) Selección del intervalo de oscilación
t_min = 9;
t_max = 12.67;
idx   = (t >= t_min) & (t <= t_max);

t_rec   = t(idx);
tita    = tita0(idx);
u       = u0(idx);
N       = numel(t_rec);
dt      = mean(diff(t_rec));  % paso de muestreo promedio

% 2) Cálculo de derivadas centradas
d1 = zeros(N,1);
d2 = zeros(N,1);
d3 = zeros(N,1);
d4 = zeros(N,1);
d5 = zeros(N,1);
du = zeros(N,1);

% Derivadas centradas
d1(3:N-2) = (tita(4:N-1) - tita(2:N-3)) / (2*dt);
du(3:N-2) = (u(4:N-1) - u(2:N-3)) / (2*dt);

d2(3:N-2) = (tita(4:N-1) - 2*tita(3:N-2) + tita(2:N-3)) / (dt^2);

d3(3:N-2) = (tita(5:N) - 2*tita(4:N-1) + 2*tita(2:N-3) - tita(1:N-4)) / (2*dt^3);

d4(4:N-3) = (tita(6:N) - 4*tita(5:N-1) + 6*tita(4:N-2) - 4*tita(3:N-3) + tita(2:N-4)) / (dt^4);

d5(5:N-4) = (tita(7:N) - 5*tita(6:N-1) + 10*tita(5:N-2) - 10*tita(3:N-4) + 5*tita(2:N-5) - tita(1:N-6)) / (dt^5);

% Recortar bordes inválidos
valid_idx = 5:N-4;
d1 = d1(valid_idx);
d2 = d2(valid_idx);
d3 = d3(valid_idx);
d4 = d4(valid_idx);
d5 = d5(valid_idx);
du = du(valid_idx);
tita = tita(valid_idx);
u = u(valid_idx);
t_fit = t_rec(valid_idx);

% 4) Estimación por mínimos cuadrados
A = [d5, d4, d3, d2, d1, tita, du, u];
theta = (A' * A) \ (A' * tita);

% 5) Coeficientes
a5 = theta(1);
a4 = theta(2);
a3 = theta(3);
a2 = theta(4);
a1 = theta(5);
a0 = theta(6);
b1 = theta(7);
b0 = theta(8);

% 6) Mostrar modelo
fprintf('\nModelo extendido (orden 5):\n');
fprintf('  %.4e d⁵θ + %.4e d⁴θ + %.4e d³θ + %.4e d²θ + %.4e dθ + %.4e θ = %.4e du + %.4e u\n\n', ...
    a5, a4, a3, a2, a1, a0, b1, b0);

% 7) Transferencia extendida
num = [b1, b0];
den = [a5, a4, a3, a2, a1, a0];
Gs = tf(num, den);

disp('Función de transferencia continua extendida G(s):');
Gs

planta_total = zpk(Gs);


% 8) Validación del modelo
tita_est = A * theta;



figure('Name','Comparación modelo - medición','NumberTitle','off');
plot(t_fit, tita, 'b', 'LineWidth', 1.5); hold on;
plot(t_fit, tita_est, 'r--', 'LineWidth', 1.5);
legend('θ real','θ estimada por modelo');
xlabel('Tiempo (s)');
ylabel('\theta');
title('Validación del modelo estimado');
grid on;



