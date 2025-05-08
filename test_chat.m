% Cargar los datos
data = load('datos_hoy.mat');
%HOLA PEDRITEN DESPUES LO HACEMOS COMO DICE SELLERIO; LE PEDI A CHAT QUE ME
%HAGA LA IDENTIFICACION.


%% agarro tita - tita0
%phi - phi_0 y los voy corriendo

%osea que me queda tita1 = tita0(1: i+1);
%tita1 = tita0(1: i+2);
%tita1 = tita0(1: i+3);


tita_x = data.mediciones.data(:,1);     % Salida
u_futuro = data.mediciones.data(:,2);   % Entrada
phi = data.mediciones.data(:,3);        % Variable adicional (no usada en el modelo por ahora)
t = data.mediciones.time;               % Vector de tiempo

% Normalizamos si es necesario
u = u_futuro(:);
y = tita_x(:);
dt = mean(diff(t));  % Tiempo de muestreo promedio

% Orden del modelo: puedes ajustarlo
na = 6;   % Número de coeficientes de la salida pasada (p.ej., 2 pares complejos => orden 4+)
nb = 3;   % Número de coeficientes de la entrada pasada

% Construcción de la matriz de regresores
N = length(y);
Phi = zeros(N - max(na, nb), na + nb);
Y = y(max(na, nb)+1:end);

for i = 1:(N - max(na, nb))
    % Salidas pasadas (con signo negativo como en ARX)
    Phi(i, 1:na) = -y(i + (na-1):-1:i)';
    % Entradas pasadas
    Phi(i, na+1:end) = u(i + (nb-1):-1:i)';
end

% Mínimos cuadrados
theta = Phi \ Y;

% Coeficientes del modelo
a = [1; theta(1:na)];
b = theta(na+1:end);

% Mostrar la función de transferencia estimada
sys_ident = tf(b', a', dt);
disp('Modelo identificado (discreto):');
sys_ident

% Convertir a continuo usando c2d (opcional, requiere System Identification Toolbox)
sys_c = d2c(sys_ident);
disp('Modelo convertido a continuo:');
sys_c

% -------------------------------------------------------------------------
% Ajuste por mínimos cuadrados de un modelo diferencial de 3er orden
% y obtención de la función de transferencia G(s)
% -------------------------------------------------------------------------

%% 1) Carga de datos
clear all;
clc;
data = load('datos_hoy.mat');
t      = data.mediciones.time;       % vector de tiempo
tita0  = data.mediciones.data(:,1);  % θ(t)
u0     = data.mediciones.data(:,3);  % u(t)

% 2) Selección del intervalo de oscilación
t_min = 9;
t_max = 12.67;
idx   = (t >= t_min) & (t <= t_max);

t_rec   = t(idx);
tita    = tita0(idx);
u       = u0(idx);
N       = numel(t_rec);
dt      = mean(diff(t_rec));  % paso de muestreo promedio

% 3) Cálculo manual de derivadas (forward‐difference)
d1    = zeros(N,1);  % dθ/dt
d2    = zeros(N,1);  % d²θ/dt²
d3    = zeros(N,1);  % d³θ/dt³
du    = zeros(N,1);  % du/dt

% primera derivada de θ y de u
for k = 1:N-1
    d1(k) = (tita(k+1) - tita(k)) ;
    du(k) = (u(k+1)    - u(k))    ;
end
d1(N) = d1(N-1);
du(N) = du(N-1);

% segunda derivada de θ
for k = 1:N-1
    d2(k) = (d1(k+1) - d1(k)) ;
end
d2(N) = d2(N-1);

% tercera derivada de θ
for k = 1:N-1
    d3(k) = (d2(k+1) - d2(k));
end
d3(N) = d3(N-1);

% 4) Construcción de la matriz de regresores y vector b
% Modelo: a3*d3 + a2*d2 + a1*d1 + a0*tita = b1*du + b0*u
A = [ d3,    d2,    d1,    tita,    du,    u  ];

%{
b = zeros(N,1);  % lado derecho homogéneo

% 5) Estimación por mínimos cuadrados (ecuación normal)
theta = (A' * A) \ (A' * b);

% Extraigo coeficientes
a3 = theta(1);
a2 = theta(2);
a1 = theta(3);
a0 = theta(4);
b1 = theta(5);
b0 = theta(6);

fprintf('\nModelo estimado:\n');
fprintf('  %.4e d³θ + %.4e d²θ + %.4e dθ + %.4e θ = %.4e du + %.4e u\n\n', ...
        a3, a2, a1, a0, b1, b0);

% 6) Poner ceros fuera del intervalo y graficar
tita_f = zeros(size(tita0));
u_f    = zeros(size(u0));
tita_f(idx) = tita;
u_f(idx)    = u;

figure('Name','Señales filtradas','NumberTitle','off');
subplot(2,1,1);
plot(t, tita_f, 'b','LineWidth',1.2); grid on;
xlabel('Tiempo (s)'); ylabel('\theta (filtrada)');
title('Salida \theta (solo intervalo de oscilación)');

subplot(2,1,2);
plot(t, u_f, 'r','LineWidth',1.2); grid on;
xlabel('Tiempo (s)'); ylabel('u (filtrada)');
title('Entrada u (solo intervalo de oscilación)');

% 7) Construcción y despliegue de G(s)
% Numerador y denominador de la TF continua
num = [b1, b0];
den = [a3, a2, a1, a0];

% Crear el modelo de espacio de estados o TF
Gs = tf(num, den);

disp('Función de transferencia continua G(s):');
Gs
%}

%%
%SIGO CON A MODIFICADA

b = [0;0;0;0;0;1]
AT = A'
cuad  = AT*A;           % 6×6
cuadi = inv(cuad);      % 6×6

X=cuadi*AT*u

% b debe ser 368×1, ¡no 6×1!
b = zeros(size(tita));  % 368×1

% multiplicación de matrices, no elemento‑a‑elemento
X = cuadi * (AT * b);   % (6×6) * (6×368 * 368×1) ⇒ 6×1

bc = A*X
e2 = (b-bc).^2
error = sum(e2)


