clc
clear all;
% Cargar archivo
data = load('datos_hoy.mat');

% Mostrar variables disponibles
whos

data_leer = load('datos_hoy.mat');   % Carga el archivo .mat
writematrix(data_leer.mediciones.data(:,1), 'tita_texto.txt'); 


tita_x = data.mediciones.data(:,1);
u_futuro = data.mediciones.data(:,2);
phi = data.mediciones.data(:,3);
t = data.mediciones.time;

% Figura 1: tita_x vs tiempo
figure;
plot(t, tita_x, 'b', 'LineWidth', 1.5);
title('Ángulo \theta_x vs Tiempo');
xlabel('Tiempo [s]');
ylabel('\theta_x [°]');
grid on;
legend('\theta_x');
set(gca, 'FontSize', 12);

% Figura 2: phi vs tiempo
figure;
plot(t, phi, 'm', 'LineWidth', 1.5);
title('Ángulo \phi vs Tiempo');
xlabel('Tiempo [s]');
ylabel('\phi [°]');
grid on;
legend('\phi');
set(gca, 'FontSize', 12);
%{
% Figura 3: u_futuro vs tiempo
figure;
plot(t, u_futuro, 'g', 'LineWidth', 1.5);
title('Control u vs Tiempo');
xlabel('Tiempo [s]');
ylabel('u');
grid on;
legend('u');
set(gca, 'FontSize', 12);
%}


