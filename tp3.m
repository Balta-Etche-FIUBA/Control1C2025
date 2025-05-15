%Ejercicio 3. 
%Sabiendo que la transferencia ser´a del tipo P = -K/s+p
%, desarrolle el modelo
%matem´atico simb´olico en torno al punto de linealizaci´on h0 = 0.45 m.
%• Haga un ajuste por cuadrados m´ınimos con la transferencia discretizada,
%asumiendo desconocimiento absoluto de los par´ametros K y p.
%• Haga un ajuste por cuadrados m´ınimos con la transferencia discretizada,
%asumiendo que s´olo se desconoce el ´area efectiva de la v´alvula de salida.
%• Utilice las funciones fit de Matlab u Octave para obtener la respuesta
%temporal discreta, y luego la transferencia continua.

%La idea es primero unicamente como datos de prueba agarrar la planta
%original y excitarlo con datos que conocemos y medir la salida para luego
%ver en una "nueva" planta desconocida poder ajustar los parametros.
clear all;
clc;
s = tf('s');
P_tp_2 = - 0.00425 / (s + 0.00257);
%Consigo la planta original del tp anterior solo para generar muestras de
%prueba, el ajuste deberia dar una planta similar a esta con los parametros
%K y p.
Ts = 1;
P_tp_dis = c2d(P_tp_2,Ts,'zoh');


N = 5000;                % Número de muestras
u = ones(N,1);         % Entrada escalón
t = (0:N-1)' * Ts;      % Vector de tiempo discreto

[num, den] = tfdata(P_tp_dis, 'v');
y = filter(num, den, u);

stairs(t, y)
xlabel('Tiempo (s)')
ylabel('Salida')
title('Respuesta de la planta discreta')

[a,b] = ajustar_mse(u,y);

function [a,b] =  ajustar_mse(u,y)
    N = length(y);
    Phi = [-y(1:N-1), u(1:N-1)];
    Y = y(2:N);
    
    if rank(Phi) < 2
        error('La matriz Phi no tiene rango completo. La entrada no es informativa.')
    end

    theta = Phi \ Y;  % más robusto que (Phi' * Phi) \ (Phi' * Y)
    a = theta(1);
    b = theta(2);
end

%%
