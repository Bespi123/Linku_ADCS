%% -------------------------------------------------------------------------
% CÁLCULO DE PROPIEDADES MÁSICAS: SISTEMA MULTICUERPO (12U + CARGAS)
% -------------------------------------------------------------------------
% Descripción:
%   Calcula el Centro de Masa (CoM) y el Tensor de Inercia total de un
%   sistema compuesto por 3 cuerpos:
%       - Cuerpo A: Satélite CubeSat 12U (Paralelepípedo)
%       - Cuerpo B: Carga útil esférica (Hueca/Sólida)
%       - Cuerpo C: Carga útil esférica secundaria (Hueca/Sólida)
%   Utiliza el Teorema de los Ejes Paralelos (Steiner) para combinar las
%   inercias respecto a un marco de referencia común.
%
% Unidades: Sistema Internacional (kg, m, kg·m²)
% -------------------------------------------------------------------------

clear; clc; close all;

%% 1. DEFINICIÓN DE PARÁMETROS FÍSICOS
error = 0.30; % Porcentaje de error

% -------------------------------------------------------------------------
% 1.1. Satélite A (Cuerpo Principal - 12U)
% Fuente: Configuración 1 (CAD/SolidWorks)
% -------------------------------------------------------------------------
m_a = 1.343382; % Masa (kg)
m_a = m_a + error*m_a;

% Dimensiones del Chasis 12U (m)
l_x_a = 226.3e-3; 
l_y_a = 226.3e-3; 
l_z_a = 340.5e-3; 

% Posición y CoM
pos_a = [0, 0, 0]; % Origen del marco de referencia (Centro Geométrico)
com_a = [0.05, -0.44, -149.03] * 1e-3; % CoM local respecto al origen geométrico

% Tensor de Inercia local respecto a SU propio CoM (kg·m²)
% Nota: Se multiplica por 1e-9 para convertir de g·mm² a kg·m²
I_a = [ 31082487.31,    -7248.29,     6857.64;
          -7248.29,  31252296.38,   -66413.94;
           6857.64,    -66413.94, 19986919.63 ] * 1e-9;

% -------------------------------------------------------------------------
% 1.2. Satélite B (Carga Útil 1 - Esfera)
% -------------------------------------------------------------------------
m_b = 5.0;  % Masa (kg)
m_b = m_b + error*m_b;
r_b = 0.1;  % Radio (m)

% Posición: Apilado sobre el eje Z del Satélite A
% (Cálculo geométrico basado en mounting plate)
pos_b = [0, 0, (67.3e-3 - l_z_a/2 + r_b)]; 

% Inercia Local (Esfera Hueca: 2/3*m*r^2 | Esfera Sólida: 2/5*m*r^2)
% Actualmente configurado como: ESFERA HUECA (Shell)
factor_b = 2/3; 
I_b = diag([factor_b*m_b*r_b^2, factor_b*m_b*r_b^2, factor_b*m_b*r_b^2]);

% -------------------------------------------------------------------------
% 1.3. Satélite C (Carga Útil 2 - Esfera Pequeña)
% -------------------------------------------------------------------------
m_c = 0.5;   % Masa (kg)
m_c = m_c + error*m_c;
r_c = 0.05;  % Radio (m)

% Posición: Apilado directamente sobre el Satélite B
pos_c = pos_b + [0, 0, r_b + r_c];

% Inercia Local (Esfera Hueca)
factor_c = 2/3;
I_c = diag([factor_c*m_c*r_c^2, factor_c*m_c*r_c^2, factor_c*m_c*r_c^2]);

%% 2. CÁLCULO DEL SISTEMA TOTAL (A + B + C)

% 2.1. Centro de Masa Total (CoM_ABC)
M_tot = m_a + m_b + m_c;

% Vectores de posición absolutos (Asumiendo que pos_a es el origen [0,0,0])
rr_a = com_a(:); % El CoM de A ya incluye su desviación interna
rr_b = pos_b(:); % Asumimos esfera homogénea -> CoM = Centro Geométrico
rr_c = pos_c(:); % Asumimos esfera homogénea -> CoM = Centro Geométrico

r_COM_TOTAL = (m_a*rr_a + m_b*rr_b + m_c*rr_c) / M_tot;

% 2.2. Tensor de Inercia Total (Teorema de Steiner / Ejes Paralelos)
% Función anónima para la matriz de transporte: J = m * ((r'r)I - rr')
P = @(r) (dot(r,r)*eye(3) - (r(:)*r(:).'));

% Suma de Inercias Locales + Transporte al CoM Total
I_COM_TOTAL = (I_a + m_a*P(rr_a - r_COM_TOTAL)) ...
            + (I_b + m_b*P(rr_b - r_COM_TOTAL)) ...
            + (I_c + m_c*P(rr_c - r_COM_TOTAL));

% 2.3. Resultados en Consola
fprintf('\n=================================================\n');
fprintf(' RESULTADOS: SISTEMA COMPLETO (A + B + C)\n');
fprintf('=================================================\n');
fprintf('Masa Total:       %.4f kg\n', M_tot);
fprintf('Centro de Masa:   [%.4f, %.4f, %.4f] m\n', r_COM_TOTAL);
fprintf('Tensor de Inercia (Respecto al CoM Total) [kg·m²]:\n');
disp(I_COM_TOTAL);

%% 3. CÁLCULO DEL SUBSISTEMA DE CARGAS (B + C)
% Útil para análisis de separación o dinámica flexible

M_tot_BC  = m_b + m_c;
r_COM_BC  = (m_b*rr_b + m_c*rr_c) / M_tot_BC;

% Inercia combinada de B y C respecto a SU propio CoM común
I_COM_BC = (I_b + m_b*P(rr_b - r_COM_BC)) ...
         + (I_c + m_c*P(rr_c - r_COM_BC));

fprintf('\n=================================================\n');
fprintf(' RESULTADOS: SUBSISTEMA CARGAS (B + C)\n');
fprintf('=================================================\n');
fprintf('Masa (B+C):       %.4f kg\n', M_tot_BC);
fprintf('Centro de Masa:   [%.4f, %.4f, %.4f] m\n', r_COM_BC);
fprintf('Tensor de Inercia (Respecto al CoM B-C) [kg·m²]:\n');
disp(I_COM_BC);

%% 4. VISUALIZACIÓN 3D
% -------------------------------------------------------------------------
figure('Name','Configuración Multicuerpo','Color','w'); 
hold on; axis equal; grid on; view(40,25);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Configuración Espacial y Centros de Masa');

% --- 4.1. Dibujar Satélite A (Caja Transparente) ---
hx = l_x_a/2; hy = l_y_a/2; hz = l_z_a/2;
V = [-hx -hy -hz; hx -hy -hz; hx hy -hz; -hx hy -hz; 
     -hx -hy  hz; hx -hy  hz; hx hy  hz; -hx hy  hz];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

patch('Vertices',V,'Faces',F, 'FaceColor',[0.2 0.4 0.8], ...
      'FaceAlpha',0.15, 'EdgeColor',[0.1 0.2 0.5], 'LineWidth',1.2);

% --- 4.2. Dibujar Satélites B y C (Esferas) ---
[xs, ys, zs] = sphere(50); 

% Satélite B
surf(r_b*xs + pos_b(1), r_b*ys + pos_b(2), r_b*zs + pos_b(3), ...
     'FaceColor',[0.9 0.3 0.3], 'FaceAlpha',0.2, 'EdgeColor','none');

% Satélite C
surf(r_c*xs + pos_c(1), r_c*ys + pos_c(2), r_c*zs + pos_c(3), ...
     'FaceColor',[0.3 0.8 0.3], 'FaceAlpha',0.2, 'EdgeColor','none');

% --- 4.3. Marcadores de Centros de Masa ---
% CoM A (Local)
p1 = plot3(com_a(1), com_a(2), com_a(3), 'bo', 'MarkerFaceColor','b', 'MarkerSize',6);
% CoM B
p2 = plot3(pos_b(1), pos_b(2), pos_b(3), 'ro', 'MarkerFaceColor','r', 'MarkerSize',6);
% CoM C
p3 = plot3(pos_c(1), pos_c(2), pos_c(3), 'go', 'MarkerFaceColor','g', 'MarkerSize',6);

% CoM TOTAL (A+B+C)
p4 = plot3(r_COM_TOTAL(1), r_COM_TOTAL(2), r_COM_TOTAL(3), ...
      'kp', 'MarkerFaceColor','y', 'MarkerSize',12, 'LineWidth',1.5);

% CoM SUBSISTEMA (B+C)
p5 = plot3(r_COM_BC(1), r_COM_BC(2), r_COM_BC(3), ...
      'ms', 'MarkerFaceColor','m', 'MarkerSize',8, 'LineWidth',1.5);

% --- 4.4. Estética Final ---
legend([p1, p2, p3, p4, p5], ...
       'CoM Satellite A', 'CoM Satellite B', 'CoM Satellite C', ...
       'CoM TOTAL (SAT A + SAT B + SAT C)', 'CoM Subsystem (SAT B + SAT C)', ...
       'Location', 'northeastoutside');

camlight headlight; lighting gouraud;