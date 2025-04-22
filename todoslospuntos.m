clear all; clc; close all;

%% 1. Planta continua original (conocida solo para generar datos)
s = tf('s');
K_real = 0.00425;
p_real = 0.00257;
P_real = -K_real / (s + p_real);

Ts = 1;  % Tiempo de muestreo
P_dis = c2d(P_real, Ts, 'zoh');  % Planta discreta

%2. Entrada y generación de salida
N = 5000;
u = ones(N,1);         % Entrada escalón
t = (0:N-1)' * Ts;

[num, den] = tfdata(P_dis, 'v');
y = filter(num, den, u);  % Simulación de planta real

% 3. Ajustar planta por mínimos cuadrados (y obtener K, p)
[K_est, p_est] = ajustar_parametros_continuos(u, y, Ts);

fprintf('K estimado: %.6f\n', K_est);
fprintf('p estimado: %.6f\n', p_est);

% 4. Simular salida de planta estimada
s = tf('s');  % Redefinir tf
P_est = -K_est / (s + p_est);
P_est_dis = c2d(P_est, Ts, 'zoh');

[num_est, den_est] = tfdata(P_est_dis, 'v');
y_est = filter(num_est, den_est, u);

%5. Graficar comparación
figure;
plot(t, y, 'b', 'DisplayName', 'Planta Real')
hold on
plot(t, y_est, 'r--', 'DisplayName', 'Planta Estimada')
xlabel('Tiempo (s)')
ylabel('Salida')
title('Comparación: Planta real vs estimada')
legend()
grid on


%% Punto 3 
% Parte principal
p_conocido = 0.00257;
Ts = 1;

% Ajustar solo K
K_est = ajustar_K_fijo_p(u, y, Ts, p_conocido);

fprintf('K estimado (con p conocido): %.6f\n', K_est);

% Simular la planta estimada
s = tf('s');
P_est = -K_est / (s + p_conocido);
P_est_dis = c2d(P_est, Ts, 'zoh');

[num_est, den_est] = tfdata(P_est_dis, 'v');
y_est = filter(num_est, den_est, u);

% Comparar gráficamente
t = (0:length(u)-1)' * Ts;
figure;
plot(t, y, 'b', 'DisplayName', 'Planta real')
hold on
plot(t, y_est, 'r--', 'DisplayName', 'Planta estimada (solo K)')
xlabel('Tiempo (s)')
ylabel('Salida')
legend()
grid on
title('Ajuste con p conocido, solo estimando K')

clear all;
clc;

s = tf('s');
Ts = 1;
%%
clear all;clc;
s=tf('s');
Ts=1;
%Punto 4

% Planta continua original solo para generar datos simulados
P_cont = -0.00425 / (s + 0.00257);

% Discretización con método de Euler (forward) => 'foh'
P_dis = c2d(P_cont, Ts, 'foh');

% Generar datos de entrada/salida simulados


N = 5000;                  % Cantidad de muestras



[num, den] = tfdata(P_dis, 'v');

nx = 10;  % al menos el orden del sistema, o un poco más
u = [zeros(nx,1); ones(N,1)];
y = filter(num, den, u);

t = (0:length(y)-1)' * Ts;
% Graficar respuesta
figure;
stairs(t, y)
xlabel('Tiempo (s)')
ylabel('Altura (salida)')
title('Respuesta simulada de la planta discreta (Euler)')

% Crear objeto iddata para identificación
data = iddata(y, u, Ts);

% Ajustar modelo discreto (estructura: OE: Output Error)
ordenNumerador = 1;
ordenDenominador = 1;
retardo = 0;

modelo_discreto = oe(data, [ordenNumerador, ordenDenominador, retardo]);

% Mostrar la transferencia identificada (discreta)
disp('Transferencia discreta ajustada:');
tf(modelo_discreto)

% Convertir a modelo continuo usando Euler (forward)
modelo_continuo = d2c(modelo_discreto, 'foh');

% Mostrar la transferencia continua equivalente
disp('Transferencia continua estimada:');
tf(modelo_continuo)

% Comparar visualmente
figure;
compare(data, modelo_discreto)
title('Comparación entre salida real y modelo ajustado (discreto)')


% Función de ajuste
function K = ajustar_K_fijo_p(u, y, Ts, p)
    % Discretizar la planta para varios valores de K, y ajustar el que minimice el error cuadrático
    N = length(y);
    t = (0:N-1)' * Ts;

    % Función de error en base a K
    error_mse = @(K_try) calcular_error(K_try, u, y, Ts, p);

    % Búsqueda del mejor K usando mínimos cuadrados
    K = fminsearch(error_mse, 0.001);  % valor inicial aproximado

end

function err = calcular_error(K, u, y, Ts, p)
    s = tf('s');
    P = -K / (s + p);
    P_dis = c2d(P, Ts, 'zoh');
    [num, den] = tfdata(P_dis, 'v');
    y_sim = filter(num, den, u);
    err = sum((y - y_sim).^2);  % error cuadrático
end

% 6. Función de ajuste
function [K, p] = ajustar_parametros_continuos(u, y, Ts)
    N = length(y);
    Phi = [-y(1:N-1), u(1:N-1)];
    Y = y(2:N);

    if rank(Phi) < 2
        error('La matriz Phi no tiene rango completo.')
    end

    theta = Phi \ Y;
    a = theta(1);
    b = theta(2);

    % Paso 1: Calcular z_p y luego p
    z_p = -a;
    if z_p <= 0 || z_p >= 1
        warning('El valor de z_p es inválido. Puede haber error en los datos o el ajuste.')
        p = NaN;
        K = NaN;
        return;
    end
    p = -log(z_p) / Ts;%convierto el polo por la discretizacion de ZoH, zp aprox 1, Ts = 1. 

    % Paso 2: Calcular K desde b y p
    K = -b * p / (1 - exp(-p * Ts));%Calculo la ganancia cuando z = p, 
end
