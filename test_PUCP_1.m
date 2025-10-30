%clear; clc; close all;

%% 1. PARÁMETROS Y CONSTANTES
disp('Configurando parámetros...');
params.mu       = 3.986e14;        % Parámetro gravitacional de la Tierra [m^3/s^2]
params.R_earth  = 6378e3;          % Radio ecuatorial de la Tierra [m]
params.J2       = 1.08263e-3;      % Coeficiente J2 de la Tierra (adimensional)
params.omega_earth = [0; 0; 7.2921E-5]; % Velocidad angular de la Tierra [rad/s]

% Parámetros de tercer cuerpo
params.mu_sun  = 1.32712440018e20; % Parámetro gravitacional del Sol [m^3/s^2]
params.mu_moon = 4.9048695e12;     % Parámetro gravitacional de la Luna [m^3/s^2]
params.R_sun   = 1.496e11;         % Distancia media Tierra-Sol [m]
params.R_moon  = 384400e3;         % Distancia media Tierra-Luna [m]
params.phi_sun  = 0;               % Fase inicial Sol [rad]
params.phi_moon = 0;               % Fase inicial Luna [rad]

% Modelo de Densidad Atmosférica (Datos de tu tabla)
alts_km = [700,675,650,625,600,575,550,525,500,475,450,425,400,375,350,325,300,275,250,225,200,175,150,125,100];
rhos_kgm3 = [3.1e-14,4.1e-14,5.7e-14,7.9e-14,1.1e-13,1.6e-13,2.4e-13,3.5e-13,5.2e-13,7.7e-13,1.2e-12,1.8e-12,2.8e-12,4.4e-12,7e-12,1.1e-11,...
       1.9e-11,3.3e-11,6e-11,1.2e-10,2.5e-10,6.3e-10,2.1e-9,1.1e-8,5.6e-7];
log_rhos = log(rhos_kgm3);
params.logRhoInterp = @(h_km) interp1(alts_km, log_rhos, h_km, 'pchip', 'extrap');
params.rhoOfH = @(h_m) exp(params.logRhoInterp(h_m/1000)); % La función espera altura en metros [m]

% PARÁMETROS DEL SATÉLITE
% Estos son necesarios para el modelo de arrastre en dynamicsModel
params.m_sat = 5.5;     % Masa del satélite [kg]
params.Cd = 1.5;        % Coeficiente de arrastre (adimensional)
params.Aeff = 0.0392699081699;      % Área efectiva de arrastre [m^2]


%% 2. CONDICIONES INICIALES (TLE)
a    = 6992e3;               % semi-major axis [m]
e    = 0.0037118;            % eccentricity
i    = deg2rad(97.799);      % inclination [rad]
RAAN = deg2rad(80.477);      % RAAN [rad]
Omega= deg2rad(65.443);      % argument of perigee [rad]
f    = deg2rad(294.681705);  % true anomaly [rad]

% Convertir elementos orbitales a vector de estado (r, v)
[r0, v0] = oe2sv(a, e, i, RAAN, Omega, f, params.mu);
y0 = [r0; v0]; % Vector de estado inicial 6x1 [r; v]


%% 3. CONFIGURAR Y EJECUTAR LA SIMULACIÓN ODE45

% Calcular el periodo orbital para un tiempo de simulación razonable
T_period = 2*pi*sqrt(a^3 / params.mu); % Periodo en segundos
fprintf('Periodo orbital: %.1f minutos\n', T_period/60);

% Simular por 2 días
orbits_num = 500;
tspan = [0, orbits_num* T_period]; % Rango de tiempo [s]

% Configurar opciones del integrador para alta precisión
% y añadir un evento para detener si el satélite reingresa
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12, ...
                 'Events', @(t,s) stopEvents(t,s,params));

% Ejecutar el integrador ode45
disp('Iniciando la propagación orbital (esto puede tardar)...');
[t_out, y_out] = ode45(@(t, state) dynamicsModel(t, state, params), tspan, y0, options);
disp('Simulación completada.');


%% 4. POST-PROCESAMIENTO Y GRÁFICOS

% --- Gráfico 1: Órbita 3D ---
figure('Name', 'Trayectoria Orbital 3D');
plot3(y_out(:,1)/1e3, y_out(:,2)/1e3, y_out(:,3)/1e3, 'b');
hold on;
title('Órbita Propagada (ECI)');
xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');
grid on;
axis equal;

% Dibujar la Tierra
[x_e, y_e, z_e] = sphere(50);
surf(x_e*params.R_earth/1e3, y_e*params.R_earth/1e3, z_e*params.R_earth/1e3, ...
     'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% --- Gráfico 2: Evolución de la Altitud ---
figure('Name', 'Evolución de la Altitud');
% Calcular magnitud del vector de posición en cada paso
r_mag = vecnorm(y_out(:, 1:3), 2, 2);
% Calcular altitud
alt = r_mag - params.R_earth;
plot(t_out / 3600, alt / 1e3); % Tiempo en horas, altitud en km
title('Altitud vs. Tiempo');
xlabel('Tiempo (horas)');
ylabel('Altitud (km)');
grid on;

% --- Gráfico 3: Posición en ECI ---
figure('Name', 'Posición ECI vs. Tiempo');

% Subplot 1: Componente X
subplot(3,1,1)
plot(t_out / 3600, y_out(:,1)/1000);
grid on;
ylabel('Posición X (km)');

% Subplot 2: Componente Y
subplot(3,1,2)
plot(t_out / 3600, y_out(:,2)/1000);
grid on;
ylabel('Posición Y (km)');

% Subplot 3: Componente Z
subplot(3,1,3)
plot(t_out / 3600, y_out(:,3)/1000);
grid on;
ylabel('Posición Z (km)');
xlabel('Tiempo (horas)'); % El xlabel solo es necesario en el último gráfico

% Añadir un título general a toda la figura (para MATLAB R2018b y posteriores)
sgtitle('Evolución de las Componentes de Posición (ECI)');

%% --------------------------------------------------------------------------
%  --- FUNCIONES LOCALES REQUERIDAS ---
%  --------------------------------------------------------------------------

function dstate = dynamicsModel(t, state, params)
% dynamicsModel - Ecuaciones de Movimiento para propagación orbital.
%
% ENTRADAS:
%   t       - Tiempo actual [s]
%   state   - Vector de estado (6x1) [m; m/s] -> [rx; ry; rz; vx; vy; vz]
%   params  - Estructura con todos los parámetros
%
% SALIDAS:
%   dstate  - Derivada del estado (6x1) [m/s; m/s^2] -> [vx; vy; vz; ax; ay; az]
%--------------------------------------------------------------------------

   % --- 1. Descomponer el vector de estado ---
   r_com = state(1:3); % Vector de posición [m]
   v_com = state(4:6); % Vector de velocidad [m/s]
   
   % Componentes de posición
   x = r_com(1);
   y = r_com(2);
   z = r_com(3);
   r_norm = norm(r_com);
   s = z / r_norm; % Seno de la latitud (z / r)

   % --- 2. Aceleración Gravitacional (Central + J2) ---
   
   % Factor J2
   factor_J2 = (3/2) * params.J2 * (params.R_earth / r_norm)^2;
   
   % Gravitación del cuerpo central (ideal)
   a_x = -params.mu * x / r_norm^3;
   a_y = -params.mu * y / r_norm^3;
   a_z = -params.mu * z / r_norm^3;
   a_grav = [a_x; a_y; a_z];
   
   % Aceleración de perturbación J2
   a_J_x = (params.mu * x / r_norm^3) * (factor_J2 * (5*s^2 - 1));
   a_J_y = (params.mu * y / r_norm^3) * (factor_J2 * (5*s^2 - 1));
   a_J_z = (params.mu * z / r_norm^3) * (factor_J2 * (5*s^2 - 3));
   a_J2 = [a_J_x; a_J_y; a_J_z];

   % --- 3. Arrastre Atmosférico (Drag) ---
   alt = r_norm - params.R_earth;
   
   % Evitar densidad negativa o error si la órbita es subterránea (en caso de error)
   if alt < 0
       rho = 0;
   else
       rho = params.rhoOfH(alt); % Obtener densidad de la función
   end
   
   % Velocidad de la atmósfera (co-rotación)
   v_atm = cross(params.omega_earth, r_com);
   % Velocidad relativa del satélite respecto a la atmósfera
   v_rel = v_com - v_atm;
   v_rel_mag = norm(v_rel);
  
   % Fórmula de aceleración de arrastre (F/m)
   if v_rel_mag > 0
       % El factor 1e-6 convierte Aeff [m^2] a [km^2] si las unidades fueran km.
       % PERO como estamos en MKS (m, kg, s), no se necesita conversión.
       a_drag = -0.5 * rho * v_rel_mag^2 * (params.Cd * params.Aeff / params.m_sat) * (v_rel / v_rel_mag);
   else
       a_drag = [0; 0; 0];
   end
  
   % --- 4. Aceleraciones de Tercer Cuerpo (Sol y Luna) ---
   
   % Sol (modelo de órbita circular simple en el ecuador)
   omega_sun = 2*pi/(365.25*86400);  % Tasa angular media del Sol [rad/s]
   theta_sun = omega_sun * t + params.phi_sun;
   r_sun = params.R_sun * [cos(theta_sun); sin(theta_sun); 0];
   % Aceleración del Sol (vector del satélite al Sol) - (vector de la Tierra al Sol)
   r_sat_sun = r_sun - r_com;
   a_sun = params.mu_sun * ( r_sat_sun / norm(r_sat_sun)^3 - r_sun/norm(r_sun)^3 );
  
   % Luna (modelo de órbita circular inclinada simple)
   omega_moon = 2*pi/(27.321582*86400);  % Tasa angular media de la Luna [rad/s]
   theta_moon = omega_moon * t + params.phi_moon;
   incl_moon = deg2rad(5.145); % Inclinación media de la órbita lunar
   
   r_moon = params.R_moon * [cos(theta_moon); 
                            sin(theta_moon)*cos(incl_moon); 
                            sin(theta_moon)*sin(incl_moon)];
   % Aceleración de la Luna
   r_sat_moon = r_moon - r_com;
   a_moon = params.mu_moon * ( r_sat_moon / norm(r_sat_moon)^3 - r_moon/norm(r_moon)^3 );
  
   % Suma de perturbaciones de tercer cuerpo
   a_third = a_sun + a_moon;
  
   % --- 5. Ensamblar la derivada de estado ---
   
   % Aceleración total
   a_total = a_grav + a_J2 + a_drag + a_third;
   
   % dstate = [dr/dt; dv/dt]
   dstate = [v_com; a_total];
end

% -------------------------------------------------------------------------

function [r_eci, v_eci] = oe2sv(a, e, i, RAAN, Omega, f, mu)
% oe2sv - Convierte elementos orbitales clásicos a vector de estado ECI.
%
% ENTRADAS (unidades MKS):
%   a, e, i, RAAN, Omega, f, mu
% SALIDAS (unidades MKS):
%   r_eci (3x1), v_eci (3x1)

    % 1. Parámetro orbital (semilatus rectum)
    p = a * (1 - e^2);

    % 2. Posición y velocidad en el sistema Perifocal (PQW)
    r_norm = p / (1 + e * cos(f));
    r_pqw = [r_norm * cos(f); r_norm * sin(f); 0];
    
    v_pqw = sqrt(mu/p) * [-sin(f); e + cos(f); 0];

    % 3. Matriz de rotación de PQW a ECI
    % (Rotación R3(RAAN) * R1(i) * R3(Omega))
    
    R3_RAAN = [cos(RAAN) -sin(RAAN) 0;
               sin(RAAN)  cos(RAAN) 0;
               0          0         1];
           
    R1_i = [1  0       0;
            0  cos(i) -sin(i);
            0  sin(i)  cos(i)];
        
    R3_Omega = [cos(Omega) -sin(Omega) 0;
                sin(Omega)  cos(Omega) 0;
                0           0          1];

    C_pqw2eci = R3_RAAN * R1_i * R3_Omega;

    % 4. Rotar vectores a ECI
    r_eci = C_pqw2eci * r_pqw;
    v_eci = C_pqw2eci * v_pqw;
end

% -------------------------------------------------------------------------

function [value, isterminal, direction] = stopEvents(t, state, params)
% stopEvents - Función de eventos de ODE para detener la simulación en reingreso.
    r_norm = norm(state(1:3));
    % value es la cantidad que se rastrea. Queremos que sea 0.
    value = r_norm - params.R_earth;
    
    % isterminal = 1 -> Detener la integración si value = 0
    isterminal = 1; 
    
    % direction = -1 -> Detectar solo si 'value' está decreciendo (cayendo)
    direction = -1;
end