parameters = simulationParameters();
results = core.runOrekitSimulation(parameters);
test_PUCP_1;

% --- Gráfico 3: Posición en ECI ---
figure('Name', 'Posición ECI vs. Tiempo');

% Subplot 1: Componente X
subplot(3,1,1)
plot(t_out / 3600, y_out(:,1)/1000); hold on
plot(results(:, 1) / 3600, results(:, 2)/1000);
grid on;
ylabel('Posición X (km)');

% Subplot 2: Componente Y
subplot(3,1,2)
plot(t_out / 3600, y_out(:,2)/1000); hold on
plot(results(:, 1) / 3600, results(:, 3)/1000);
grid on;
ylabel('Posición Y (km)');

% Subplot 3: Componente Z
subplot(3,1,3)
plot(t_out / 3600, y_out(:,3)/1000); hold on
plot(results(:, 1) / 3600, results(:, 4)/1000);
grid on;
ylabel('Posición Z (km)');
xlabel('Tiempo (horas)'); % El xlabel solo es necesario en el último gráfico

% Añadir un título general a toda la figura (para MATLAB R2018b y posteriores)
sgtitle('Evolución de las Componentes de Posición (ECI)');



%% 5. CÁLCULO DE LA DIFERENCIA (ERROR) ENTRE SIMULACIONES

disp('Calculando la diferencia entre ode45 y Orekit...');

% 1. Interpolar los resultados de Orekit (results) en los puntos de tiempo de ode45 (t_out)
%    Esto es necesario porque los pasos de tiempo no son idénticos.
%    interp1(Tiempo_Original, Datos_Originales, Tiempos_Nuevos)

orekit_x_interp = interp1(results(:, 1), results(:, 2), t_out);
orekit_y_interp = interp1(results(:, 1), results(:, 3), t_out);
orekit_z_interp = interp1(results(:, 1), results(:, 4), t_out);

% Ensamblar el vector de posición interpolado de Orekit
pos_orekit_interp = [orekit_x_interp, orekit_y_interp, orekit_z_interp];

% 2. Obtener el vector de posición de ode45
%    (Asumiendo que y_out usa las mismas unidades [metros] que results)
pos_ode45 = y_out(:, 1:3);

% 3. Calcular el vector de diferencia (error) en metros
diff_vector_m = pos_ode45 - pos_orekit_interp;

% 4. Calcular la magnitud del error (distancia euclidiana) en cada paso
%    Esto nos da la distancia total en metros entre las dos predicciones
diff_magnitude_m = vecnorm(diff_vector_m, 2, 2);

% --- 6. GRÁFICO DE LA DIFERENCIA ---
figure('Name', 'Diferencia de Posición (ode45 vs Orekit)');
plot(t_out / 3600, diff_magnitude_m);
grid on;
title('Diferencia de Posición (Magnitud) vs. Tiempo');
xlabel('Tiempo (horas)');
ylabel('Diferencia (metros)');

% --- 7. MOSTRAR ESTADÍSTICAS EN CONSOLA ---
fprintf('\n--- Estadísticas de Diferencia (ode45 - Orekit) ---\n');
fprintf('Diferencia media:     %.2f metros\n', mean(diff_magnitude_m));
fprintf('Diferencia máxima:    %.2f metros\n', max(diff_magnitude_m));
fprintf('Diferencia final:     %.2f metros (al final de la simulación)\n', diff_magnitude_m(end));