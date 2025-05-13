%% -------------------------------------------------------------
%   Ajuste por mínimos cuadrados de un modelo de 3er orden
%   con recorte y centrado de señales
%% -------------------------------------------------------------
clearvars; close all; clc

%% 1) Cargo datos
S     = load('datos_hoy.mat');
t_all = S.mediciones.time;           % vector tiempo completo
theta0= S.mediciones.data(:,1);      % θ(t)
phi0  = S.mediciones.data(:,3);      % φ(t)

%% 2) Calculo punto de equilibrio PRE-escalón (antes de t=9s)
t_step = 9;  
idx_eq = t_all < t_step;
phi_eq   = mean(phi0(idx_eq));       % ≈70°
theta_eq = mean(theta0(idx_eq));     % posición inicial del péndulo

%% 3) Recorte al tramo del escalón (70→110°, t=9…12.67s)
t_min = 9;  
t_max = 12.67;
idx   = (t_all >= t_min) & (t_all <= t_max);
idx1 = (t_all>= t_min);
t_rec   = t_all(idx);
u_raw   = phi0(idx);
y_raw   = theta0(idx);

% Remuestreo uniforme en Ts
Ts = median(diff(t_rec));
t  = (0:length(t_rec)-1)' * Ts;

%% 4) Centro las señales (demean) para quedarnos solo con la dinámica
u = u_raw - phi_eq;    % ahora oscila de 0→40°
y = y_raw - theta_eq;  % dinámica pura sin offset

%% 5) Cálculo de derivadas centradas
N = numel(t);
d1 = zeros(N,1);
d2 = zeros(N,1);
d3 = zeros(N,1);
du = zeros(N,1);

% derivada primera (centrada)
d1(2:N-1) = (y(3:N) - y(1:N-2)) / (2*Ts);
du(2:N-1)= (u(3:N) - u(1:N-2)) / (2*Ts);

% derivada segunda
d2(2:N-1) = (y(3:N) - 2*y(2:N-1) + y(1:N-2)) / (Ts^2);

% derivada tercera (centrada, requiere más puntos)
d3(3:N-2) = ( y(5:N) - 2*y(4:N-1) + 2*y(2:N-3) - y(1:N-4) ) / (2*Ts^3);

% recorto índices inválidos
vidx = 3:N-2;
d1 = d1(vidx); d2 = d2(vidx); d3 = d3(vidx);
du = du(vidx); y = y(vidx); u = u(vidx);
t_fit = t(vidx);

%% 6) Formo regresión y ajusto por Mínimos Cuadrados
% matriz de regresores [d³y, d²y, d¹y, y, d¹u, u]
A = [d3, d2, d1, y, du, u];
theta = (A'*A)\(A'*y);

% extraigo coeficientes
a3 = theta(1); a2 = theta(2); a1 = theta(3); a0 = theta(4);
b1 = theta(5); b0 = theta(6);

fprintf('\nModelo estimado:\n');
fprintf('  %.3e·d³θ + %.3e·d²θ + %.3e·dθ + %.3e·θ = %.3e·du + %.3e·u\n\n', ...
        a3, a2, a1, a0, b1, b0);

%% 7) Función de transferencia continua
num = [b1, b0];
den = [a3, a2, a1, a0];
Gs = tf(num, den);
    
disp('Función de transferencia G(s):');
Gs
figure('Name','Polos y ceros de G(s)','NumberTitle','off');
pzmap(Gs); grid on;

%% 8) Validación en el mismo intervalo recortado
y_est = A*theta;
figure('Name','Validación local (9–12.67s)','NumberTitle','off');
plot(t_fit, y, 'b','LineWidth',1.5); hold on;
plot(t_fit, y_est, 'r--','LineWidth',1.5);
legend('θ real (centrada)','θ estimada');
xlabel('Tiempo (s)'); ylabel('θ_{dev} (°)');
title('Comparación en intervalo de identificación');
grid on;

%% 9) Validación extendida (simulación completa)
% Simulo desde t=3s para toda la medición
s = tf('s');
% usa tu Gs estimado arriba
y_full_est = lsim(Gs, u_raw, t_rec);

figure('Name','Validación extendida (simulación)','NumberTitle','off');
plot(t_rec, y_raw - theta_eq, 'b','LineWidth',1.2); hold on;
plot(t_rec, y_full_est, 'r--','LineWidth',1.2);
legend('θ real centrada','θ simulada');
xlabel('Tiempo (s)'); ylabel('θ_{dev} (°)');
title('Validación de G(s) en todo el tramo 9–12.67s');
grid on;

%%
%% -------------------------------------------------------------
% 9) Estimación y simulación con 3 ceros y 4 polos (dos pares de polos)
%% -------------------------------------------------------------

% Aseguro que u, y y t están definidos y en double
u = double(u(:));
y = double(y(:));
% 't' y 'Ts' deben venir del script anterior

% Creo el objeto iddata local centrado
data_id_local = iddata(y, u, Ts, 'Domain','Time');

% Especifico orden de modelo
n_p = 4;   % número de polos (dos pares)
n_z = 2;   % número de ceros

% Estimo la función de transferencia continua
sys_tf2 = tfest(data_id_local, n_p, n_z);

N = min(length(t), length(u));
t = t(1:N);
u = u(1:N);

% Simulo la respuesta en el mismo intervalo
y_sim = lsim(sys_tf2, u, t);

% Muestro polos y ceros
figure('Name','TFest 4 polos, 3 ceros','NumberTitle','off');
pzmap(sys_tf2); grid on;
title('Polos y ceros: modelo 3 ceros, 4 polos');
disp('TF continua estimada con tfest:');
sys_tf2

% Comparo la salida real vs. la simulada
figure('Name','Comparativa tfest 4p3z','NumberTitle','off');
plot(t,    y,     'b',  'LineWidth',1.5); hold on;
plot(t,    y_sim, 'r--','LineWidth',1.5);
legend('\theta real (centrada)', '\theta tfest 4p3z','Location','Best');
xlabel('Tiempo (s)');
ylabel('\theta_{dev} (°)');
title('Ajuste del modelo 3 ceros, 4 polos');
grid on;


%% -------------------------------------------------------------
% 10) Validación extendida: desde t = 3 s hasta el final
%% -------------------------------------------------------------

% Extraigo señales completas desde t = 3 s
idx_val = t_all >= 3;
t_val   = t_all(idx_val);
u_val   = phi0(idx_val);        % entrada real
y_val   = theta0(idx_val);      % salida real

% Simulo salida usando sys_tf2
y_model_val = lsim(sys_tf2, u_val, t_val);

% Grafico comparación
figure('Name','Validación extendida desde t=3s','NumberTitle','off');
plot(t_val, y_val,        'b',  'LineWidth',1.5); hold on;
plot(t_val, y_model_val,  'r--','LineWidth',1.5);
legend('\theta real','\theta estimada (tfest)','Location','Best');
xlabel('Tiempo (s)');
ylabel('\theta (°)');
title('Validación extendida del modelo (desde t = 3 s)');
grid on;

%% -------------------------------------------------------------
% 11) Representación en formato cero-polo-ganancia (ZPK)
%% -------------------------------------------------------------

% Conversión a zpk
planta_zpk = zpk(sys_tf2);

% Mostrar estructura ZPK
disp('Modelo en forma cero-polo-ganancia:');
planta_zpk

% Gráfico más visual de ceros y polos
figure('Name','ZPK - Polos y Ceros','NumberTitle','off');
pzmap(planta_zpk); grid on;
title('Ubicación de polos y ceros en el plano s');




%% -------------------------------------------------------------
% 12) Diseño del controlador, con proporcional.
%% -------------------------------------------------------------

P = zpk([0.19, 0], [-1.0135 + 15.2396i, -1.0135 - 15.2396i, -30.5063 + 91.3426i, -30.5063 - 91.3426i], 12062);



%C = zpk([-1.0135 + 15.2396i, -1.0135 - 15.2396i, -30.5063 + 91.3426i, -30.5063 - 91.3426i] , [0 0 -2.742 -1000] ,db2mag(49 -15))
%C = zpk([-1 -1 -30 -30] , [0 -2.742 -100 -100] ,db2mag(1.21 + 0.629))

%C = zpk([-10] , [0 0] ,db2mag(29));
C_Prop = db2mag(60 - 62.3);

L_prop = P*C_Prop;
Ts = 0.01
%DIGITALIZO
C_digital = c2d(C, Ts, 'tustin');

%saco los coeficientes para armar la ec en diferencias
[NUM,DEN] = tfdata(C_digital, 'v')

L = minreal(P * C)
figure;
rlocus(L)


figure;
bode(L_prop)

figure;
T = L_prop/(1+L_prop);

step(T);



%%
P1 = zpk([0.19, 0], [-1.0135 + 15.2396i, -1.0135 - 15.2396i, -30.5063 + 91.3426i, -30.5063 - 91.3426i], 12062);

k_pi = db2mag(-2);

C_PI = zpk([-10],[0],k_pi);

L_PI = P1*C_PI;

T = L_PI/(1+L_PI);

figure;
step(T);

figure;
bode(L_PI);

%% d

Kp = 0.02;
Kd = 0.005;
tau = 0.01;
C_pd = Kp + tf([Kd, 0], [tau, 1]);
C_lead = tf([1 2], [1 20]);
C_total = C_lead * C_pd;

L = C_total * P;
T = feedback(L, 1);

% Gráficos
figure;
step(T, 10); % Mostrar hasta 10 segundos
grid on;

figure;
bode(L);

% Ejemplo de uso de sisotool para ajuste interactivo

