%% 2. Controlador PI suave
K = db2mag(29);
C = zpk([-10], [0 0], K);

%% 3. Laço abierto y cerrado
L = minreal(C * P);
T = feedback(L, 1);      % Función de transferencia de referencia→salida
S = minreal(1/(1 + L));  % Función de sensibilidad (perturbación→salida)

%% 4. Simulaciones originales
figure('Name','Respuesta a escalón (T)','NumberTitle','off');
step(T, 5);
title('Respuesta a escalón SIN filtro de referencia');
ylabel('\theta (°)');
grid on;

figure('Name','Respuesta al impulso (T)','NumberTitle','off');
impulse(T, 5);
title('Respuesta al impulso del sistema en lazo cerrado');
xlabel('Tiempo (s)');
ylabel('\theta (°)');
grid on;

%% 5. Simulación de perturbación de 20° (escalón)
figure('Name','Rechazo a perturbación','NumberTitle','off');
step(60* S, 5);            % escalón de salida de 20°
title('Rechazo a perturbación de 20° sobre la salida');
xlabel('Tiempo (s)');
ylabel('\theta (°)');
grid on;
