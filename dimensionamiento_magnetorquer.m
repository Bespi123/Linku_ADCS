%% ==========================================
%  DIMENSIONAMIENTO DE MAGNETORQUERS (Cuerpo ABC)
%  ==========================================
%  Requiere primero el cálculo de I_COM_TOTAL
inertiaCalculus;

% 1. Parámetros de Diseño
omega_0_deg = 30;           % Tasa de giro inicial (Worst case: 30 deg/s)
t_orbits    = 4;            % En cuántas órbitas quieres estar estable
h_orbit_km  = 600;          % Altitud aproximada

% 2. Constantes Físicas
Re = 6378e3;
mu_earth = 3.986e14;
B0 = 3.12e-5; % Tesla (Campo en ecuador ~31000 nT)

% 3. Cálculos Derivados
R_sat = Re + h_orbit_km*1000;
Period = 2*pi*sqrt(R_sat^3/mu_earth);
t_target_sec = t_orbits * Period;
B_avg = B0 * (Re/R_sat)^3; % Campo promedio conservador

% Obtener la inercia máxima de tu ensamblaje
I_max = max(diag(I_COM_TOTAL)); 

% Momento Angular Inicial a disipar (h = I * w)
omega_0_rad = deg2rad(omega_0_deg);
h_initial = I_max * omega_0_rad;

% 4. Dipolo Requerido (Fórmula de Sizing)
% Factor 2 extra por ineficiencia geométrica del campo magnético
% Factor 2 extra porque el B-Dot no es 100% eficiente
SafetyFactor = 4; 

m_required = (h_initial * SafetyFactor) / (B_avg * t_target_sec);

fprintf('\n=========================================\n');
fprintf(' RESULTADOS DE DIMENSIONAMIENTO ACTUADOR (CUERPO ABC\n');
fprintf('=========================================\n');
fprintf('Masa Total:           %.2f kg\n', M_tot);
fprintf('Inercia Máxima:       %.4f kg·m²\n', I_max);
fprintf('Velocidad Inicial:    %d deg/s\n', omega_0_deg);
fprintf('Tiempo Objetivo:      %.1f horas (%d órbitas)\n', t_target_sec/3600, t_orbits);
fprintf('-----------------------------------------\n');
fprintf('Dipolo Mínimo (Teórico):  %.3f Am²\n', m_required/SafetyFactor);
fprintf('Dipolo RECOMENDADO:       %.3f Am²\n', m_required);
fprintf('=========================================\n');

% Sugerencia de bobina (si lo fabricas tú)
voltaje = 5; % V
power   = 1.0; % W (disponible para ADCS)
current = power / voltaje;
area_estimada = 0.10 * 0.10; % 10x30 cm (cara del 12U)

n_vueltas = m_required / (current * area_estimada);
fprintf('Para fabricarlo en una cara de %.2f m² con %.1f V / %.1f W:\n', area_estimada, voltaje, power);
fprintf('  -> Corriente: %.2f A\n', current);
fprintf('  -> Vueltas:   %d vueltas aprox.\n', ceil(n_vueltas));


%% ==========================================
%  DIMENSIONAMIENTO DE MAGNETORQUERS (Cuerpo BC)
%  ==========================================
%  Requiere primero el cálculo de I_COM_BC

% 1. Parámetros de Diseño
omega_0_deg = 30;           % Tasa de giro inicial (Worst case: 30 deg/s)
t_orbits    = 4;            % En cuántas órbitas quieres estar estable
h_orbit_km  = 600;          % Altitud aproximada

% 2. Constantes Físicas
Re = 6378e3;
mu_earth = 3.986e14;
B0 = 3.12e-5; % Tesla (Campo en ecuador ~31000 nT)

% 3. Cálculos Derivados
R_sat = Re + h_orbit_km*1000;
Period = 2*pi*sqrt(R_sat^3/mu_earth);
t_target_sec = t_orbits * Period;
B_avg = B0 * (Re/R_sat)^3; % Campo promedio conservador

% Obtener la inercia máxima de tu ensamblaje
I_max = max(diag(I_COM_BC)); 

% Momento Angular Inicial a disipar (h = I * w)
omega_0_rad = deg2rad(omega_0_deg);
h_initial = I_max * omega_0_rad;

% 4. Dipolo Requerido (Fórmula de Sizing)
% Factor 2 extra por ineficiencia geométrica del campo magnético
% Factor 2 extra porque el B-Dot no es 100% eficiente
SafetyFactor = 4; 

m_required = (h_initial * SafetyFactor) / (B_avg * t_target_sec);

fprintf('\n=========================================\n');
fprintf(' RESULTADOS DE DIMENSIONAMIENTO ACTUADOR\n');
fprintf('=========================================\n');
fprintf('Masa Total:           %.2f kg\n', M_tot_BC);
fprintf('Inercia Máxima:       %.4f kg·m²\n', I_max);
fprintf('Velocidad Inicial:    %d deg/s\n', omega_0_deg);
fprintf('Tiempo Objetivo:      %.1f horas (%d órbitas)\n', t_target_sec/3600, t_orbits);
fprintf('-----------------------------------------\n');
fprintf('Dipolo Mínimo (Teórico):  %.3f Am²\n', m_required/SafetyFactor);
fprintf('Dipolo RECOMENDADO:       %.3f Am²\n', m_required);
fprintf('=========================================\n');

% Sugerencia de bobina (si lo fabricas tú)
voltaje = 5; % V
power   = 1.5; % W (disponible para ADCS)
current = power / voltaje;
area_estimada = 0.03 * 0.03; % 3x3 cm (cara del cuerpo B)

n_vueltas = m_required / (current * area_estimada);
fprintf('Para fabricarlo en una cara de %.2f m² con %.1f V / %.1f W:\n', area_estimada, voltaje, power);
fprintf('  -> Corriente: %.2f A\n', current);
fprintf('  -> Vueltas:   %d vueltas aprox.\n', ceil(n_vueltas));