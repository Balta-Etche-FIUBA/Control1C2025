clc; clear;

%% 1. Planta estimada (la que definiste)
P = zpk([-2.742, 0], ...
        [-1.0135 + 15.2396i, -1.0135 - 15.2396i, -30.5063 + 91.3426i, -30.5063 - 91.3426i], ...
        12062);

%% 2. Controlador proporcional derivativo suave
% Este controlador lo ajustaste con un K más bajo para tener buen margen de fase
K = db2mag(29);    % Ganancia ajustada por margen de fase
C = zpk([-10], [0 0], K);  % Controlador tipo PI (cero en -1, polo en 0)

%% 3. Sistema en lazo cerrado
L = minreal(C * P);
T = feedback(L, 1);
%% 5. Simulaciones

figure('Name','Step sin filtro vs con filtro','NumberTitle','off');

% Respuesta sin filtro (entrada escalón directa)
subplot(1,1,1);
step(T, 5);  % 5 segundos
title('Respuesta a escalón SIN filtro de referencia');
ylabel('\theta (°)');
grid on;

%%
figure('Name','Respuesta al impulso','NumberTitle','off');
impulse(T, 5);  % 5 segundos
title('Respuesta al impulso del sistema en lazo cerrado');
xlabel('Tiempo (s)');
ylabel('\theta (°)');
grid on;

