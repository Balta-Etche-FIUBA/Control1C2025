%% Estimación de G(s) = θ(s)/φ(s) sin fijar el término constante a0
clear; clc;

%% 1) Carga de datos y recorte
data   = load('datos_hoy.mat');
t      = data.mediciones.time;
tita0  = data.mediciones.data(:,1);  % θ(t)
phi0   = data.mediciones.data(:,3);  % φ(t)

t_min = 9;
t_max = 12.67;
idx   = (t >= t_min) & (t <= t_max);

tita  = tita0(idx);
phi   = phi0(idx);
t_rec = t(idx);
N     = numel(t_rec);
dt    = mean(diff(t_rec));

%% 2) Cálculo de derivadas por forward‐difference
d1    = zeros(N,1);  % θ'
d2    = zeros(N,1);  % θ''
d3    = zeros(N,1);  % θ'''
dphi  = zeros(N,1);  % φ'

for k = 1:N-1
    d1(k)   = (tita(k+1) - tita(k)) / dt;
    d2(k)   = (d1(k+1)   - d1(k))   / dt;
    d3(k)   = (d2(k+1)   - d2(k))   / dt;
    dphi(k) = (phi(k+1)  - phi(k))  / dt;
end
% Replicar último valor para completar
d1(N)   = d1(N-1);
d2(N)   = d2(N-1);
d3(N)   = d3(N-1);
dphi(N) = dphi(N-1);

%% 3) Planteo de la regresión con denominador monico (a3 = 1)
% Ecuación:  d³θ + a2·d²θ + a1·dθ + a0·θ = b1·dφ + b0·φ
% =>  a2·d2 + a1·d1 + a0·tita - b1·dphi - b0·phi = -d3

M     = [ d2,    d1,    tita,   -dphi,  -phi ];
y_vec = -d3;

%% 4) Estimación por mínimos cuadrados
x = M \ y_vec;

% Extraer coeficientes
a3 = 1;        % fijado para eliminar escala trivial
a2 = x(1);
a1 = x(2);
a0 = x(3);
b1 = x(4);
b0 = x(5);

%% 5) Construcción de la función de transferencia continua
num = [ b1,  b0 ];
den = [ a3,  a2,  a1,  a0 ];
Gs  = tf(num, den);

%% 6) Mostrar resultados
fprintf('\nEcuación diferencial estimada:\n');
fprintf('  d³θ + %.4e·d²θ + %.4e·dθ + %.4e·θ = %.4e·dφ + %.4e·φ\n', ...
        a2, a1, a0, b1, b0);

disp('Función de transferencia continua G(s) = θ(s)/φ(s):');
disp(Gs);
