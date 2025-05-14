clc
clear all;
%1) Cargo datos
S     = load('datos_hoy.mat');
t_all = S.mediciones.time;           % vector tiempo completo
theta0= S.mediciones.data(:,1);      % θ(t)
phi0  = S.mediciones.data(:,3);      % φ(t)

%2) Calculo punto de equilibrio PRE-escalón (antes de t=9s)
t_step = 9;  
idx_eq = t_all < t_step;
phi_eq   = mean(phi0(idx_eq));       % ≈70°
theta_eq = mean(theta0(idx_eq));     % posición inicial del péndulo

% 3) Recorte al tramo del escalón (70→110°, t=9…12.67s)
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

% 4) Centro las señales (demean) para quedarnos solo con la dinámica
u = u_raw - phi_eq;    % ahora oscila de 0→40°
y = y_raw - theta_eq;  % dinámica pura sin offset



%% -------------------------------------------------------------
% 5) Estimación y simulación con 3 ceros y 4 polos (dos pares de polos)
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
% 6) Validación extendida: desde t = 3 s hasta el final
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
xlim([9, 25]);
legend('\theta real','\theta estimada (tfest)','Location','Best');
xlabel('Tiempo (s)');
ylabel('\theta (°)');
title('Validación extendida del modelo (desde t = 9 s)');
grid on;

%% -------------------------------------------------------------
% 7) Representación en formato cero-polo-ganancia (ZPK)
%% -------------------------------------------------------------

% Conversión a zpk
planta_zpk = zpk(sys_tf2);

% Mostrar estructura ZPK
disp('Modelo en forma cero-polo-ganancia:');
planta_zpk




%% -------------------------------------------------------------
% 8) Diseño del controlador, con proporcional.
%% -------------------------------------------------------------

P = zpk([0.19, 0], [-1.0135 + 15.2396i, -1.0135 - 15.2396i, -30.5063 + 91.3426i, -30.5063 - 91.3426i], 12062);

%C = zpk([-10] , [0 0] ,db2mag(29));
C_prop = db2mag(2.3);

L_prop = P*C_prop;
Ts = 0.01

%DIGITALIZO

%C_digital = c2d(C_Prop, Ts, 'tustin');

%saco los coeficientes para armar la ec en diferencias
%[NUM,DEN] = tfdata(C_digital, 'v')

L_prop = minreal(P * C_prop)
figure;
rlocus(L_prop)
title('ROOT LOCUS L_{PROP})');

figure;
bode(L_prop)
title('BODE L_{PROP})');

figure;
T_prop = L_prop/(1+L_prop);

step(T_prop);
title('RESPUESTA AL ESCALON T_{PROP})');




%% -------------------------------------------------------------
% 9) Diseño del controlador, proporcional integral.
%% -------------------------------------------------------------
P1 = zpk([0.19, 0], [-1.0135 + 15.2396i, -1.0135 - 15.2396i, -30.5063 + 91.3426i, -30.5063 - 91.3426i], 12062);

k_pi = db2mag(-2);

C_PI = zpk([-10],[0],k_pi);

L_PI = minreal(P1*C_PI);

T = L_PI/(1+L_PI);

figure;
step(T);
title('RESPUESTA AL ESCALON T_{PI})');

figure;
bode(L_PI);
title('BODE L_{PI})');

figure;
rlocus(L_PI)
title('ROOT LOCUS L_{PI})');



