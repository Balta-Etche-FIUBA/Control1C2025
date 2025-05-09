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

% 2) Cálculo de derivadas (centradas para mayor precisión)
d1 = zeros(N,1);
d2 = zeros(N,1);
d3 = zeros(N,1);
du = zeros(N,1);

% Primera derivada (centrada)
d1(2:N-1) = (tita(3:N) - tita(1:N-2)) / (2*dt);
du(2:N-1) = (u(3:N) - u(1:N-2)) / (2*dt);

% Segunda derivada
d2(2:N-1) = (tita(3:N) - 2*tita(2:N-1) + tita(1:N-2)) / (dt^2);

% Tercera derivada
d3(3:N-2) = (tita(5:N) - 2*tita(4:N-1) + 2*tita(2:N-3) - tita(1:N-4)) / (2*dt^3);

% 3) Recorte de datos inválidos (bordes)
valid_idx = 3:N-2;
d1 = d1(valid_idx);
d2 = d2(valid_idx);
d3 = d3(valid_idx);
du = du(valid_idx);
tita = tita(valid_idx);
u = u(valid_idx);
t_fit = t_rec(valid_idx);

% 4) Construcción de la matriz de regresores y estimación
A = [d3, d2, d1, tita, du, u];     % cada fila es un instante de tiempo
theta = (A' * A) \ (A' * tita);   % salida en función de entrada

% 5) Extraer coeficientes
a3 = theta(1);
a2 = theta(2);
a1 = theta(3);
a0 = theta(4);
b1 = theta(5);
b0 = theta(6);

% 6) Mostrar el modelo
fprintf('\nModelo estimado:\n');
fprintf('  %.4e d³θ + %.4e d²θ + %.4e dθ + %.4e θ = %.4e du + %.4e u\n\n', ...
        a3, a2, a1, a0, b1, b0);

% 7) Crear función de transferencia continua
num = [b1, b0];
den = [a3, a2, a1, a0];
Gs = tf(num, den);

disp('Función de transferencia continua G(s):');
Gs

planta_total = zpk(Gs)

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



