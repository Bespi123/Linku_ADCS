clear; clc; close all;

%% 1) PARÁMETROS DEL SATÉLITE (LINKU - 12U)
% sat.Is = [0.103018304995791, -0.000007224537238, 0.000015479696200;
%          -0.000007224537238,  0.103187907740666, -0.000142288034562;
%           0.000015479696200, -0.000142288034562,  0.054153798020147];
% sat.mass = 6.843382000000000; % kg

%% 1) PARÁMETROS DEL SATÉLITE (12U STANDAR)
Ixx = 0.2785;
Iyy = 0.2792; % Ligeramente diferente a Ixx por asimetría interna
Izz = 0.1705;

% Productos de inercia (típicamente 1-5% de los principales)
Ixy = -0.005; 
Ixz =  0.003; 
Iyz =  0.004;

sat.Is = [Ixx,  Ixy,  Ixz;
          Ixy,  Iyy,  Iyz;
          Ixz,  Iyz,  Izz];

sat.mass = 20.0;

%% Actuadores: Magnetorquers (MQ800 Model)
actuators.magnetorquer.power            = 360e-3;       % W
actuators.magnetorquer.MaxPower         = 360e-3;       % W
actuators.magnetorquer.voltage          = 5;            % V
actuators.magnetorquer.dimensions       = [70,10,10];   % mm (l,w,h)
actuators.magnetorquer.nominalDipole    = 0.2;          % A m^2
actuators.magnetorquer.maxNominalDipole = 0.2;          % A m^2

% Área (m^2) usando mm -> m: A = (h*l) [mm^2] * 1e-6
actuators.magnetorquer.A  = actuators.magnetorquer.dimensions(2) * actuators.magnetorquer.dimensions(3) * 1e-6;
% Estimación de número de vueltas "equivalente" (ojo: modelo simplificado)
actuators.magnetorquer.n  = actuators.magnetorquer.nominalDipole * actuators.magnetorquer.voltage / ...
                            (actuators.magnetorquer.A * actuators.magnetorquer.power);
actuators.magnetorquer.maxCurrent = actuators.magnetorquer.maxNominalDipole / actuators.magnetorquer.voltage;

%% Sensores: Magnetómetro
sensors.mag.desvEst = 50.00e-9; % T (ruido)
sensors.mag.res     = 31.25e-9; % T (resolución)
sensors.mag.normDesvEst = 0.5;

%% 2) PARÁMETROS AMBIENTALES Y ORBITALES
earth.Radius       = 6378e3;
earth.Mass         = 5.972e24;
earth.GravityConst = 6.674e-11;
earth.mu           = earth.GravityConst * earth.Mass;

orbit.semiMajorAxis = 6992e3;
orbit.eccentricity = 0.0037118;
orbit.inclination = 97.799;
orbit.rightAscensionOfAscendingNode = 80.477;
orbit.argumentOfPeriapsis = 65.443;
orbit.trueAnomaly = 294.681705;
orbit.period      = 2*pi/sqrt(earth.mu) * orbit.semiMajorAxis^(3/2);
orbit.vcircular   = sqrt(earth.mu/orbit.semiMajorAxis);

settings.B_earth_field.model_epoch = '2025';
settings.B_earth_field.decimal_year = 2025;
settings.wgs84 = wgs84Ellipsoid;

% External disturbances
disturbance = @(t) simplifiedDisturbances(t);  % 

%% 3) CONDICIONES INICIALES
initial.attitude.rpy0_deg = [10, -10, 5];
initial.attitude.rpy0_rad = deg2rad(initial.attitude.rpy0_deg');
initial.attitude.q0123_0  = eul2quat(initial.attitude.rpy0_rad')';  % scalar-first

initial.omega.omega0_x = deg2rad(10); %
initial.omega.omega0_y = deg2rad(-10);
initial.omega.omega0_z = deg2rad(10);

%% 4) CONFIGURACIÓN DE SIMULACIÓN
settings.startTime = datetime(2025,10,10,23,30,00);
settings.startTime.TimeZone = "UTC";

settings.number_of_orbits = 5;
settings.sample_step = 100;
settings.t_final = orbit.period * settings.number_of_orbits;
settings.orbit_period = orbit.period;                 
settings.X0 = [initial.attitude.q0123_0;
               initial.omega.omega0_x;
               initial.omega.omega0_y;
               initial.omega.omega0_z;
               0; % Integral error X
               0; % Integral error Y
               0];% Integral error Z

%% 5) Configuracion de computadora de abordo
state_ant = 0;

%% Escenario Satelital (SGP4)
sc = satelliteScenario(settings.startTime, settings.startTime + seconds(settings.t_final), settings.sample_step);
sat.satSGP4 = satellite(sc, orbit.semiMajorAxis, orbit.eccentricity, orbit.inclination, ...
        orbit.rightAscensionOfAscendingNode, orbit.argumentOfPeriapsis, orbit.trueAnomaly);

%% 5) PRE-CÁLCULO DEL ENTORNO (TABLA)
disp('Pre-calculando entorno...');
[pos_eci, vel_eci, time_array] = states(sat.satSGP4, "CoordinateFrame", "inertial");
env.Pos_inertial = pos_eci; % 3xN
env.Vel_inertial = vel_eci;
env.time = seconds(time_array - settings.startTime);  % Nx1 (seconds)

time_array_utc = datevec(time_array);
pos_eci_t = pos_eci'; % Nx3

% ECI -> ECEF (loop, porque eci2ecef es por muestra)
r_ecef = NaN(3, size(pos_eci_t,1));
for i = 1:size(pos_eci_t,1)
    r_ecef(:,i) = eci2ecef(time_array_utc(i,:), pos_eci_t(i,:));
end

% ECEF -> LLA
lla = ecef2lla(r_ecef', 'WGS84');
lat = lla(:,1);
lon = lla(:,2);
alt = lla(:,3);

% Campo magnético (wrldmagm) -> entrega B en NED (nT)
N = size(pos_eci_t,1);
B_ned = NaN(3, N);
dip   = NaN(N, 1);

for j = 1:N
    dyears = decimale_vector(datevec(time_array(j)));   % <-- j (no i)
    [B_ned(:,j), ~, ~, dip(j), ~] = wrldmagm(alt(j), lat(j), lon(j), dyears, '2025');
end

% ---- Convertir B: NED -> ECEF -> ECI (sin dcmeci2ecef) ----
lat_rad = deg2rad(lat);
lon_rad = deg2rad(lon);

B_eci = NaN(3, N); % nT

for k = 1:N
    phi = lat_rad(k);
    lam = lon_rad(k);

    % (ECEF <- NED)
    n_hat = [-sin(phi)*cos(lam);
             -sin(phi)*sin(lam);
              cos(phi)];
    e_hat = [-sin(lam);
              cos(lam);
              0];
    d_hat = [-cos(phi)*cos(lam);
             -cos(phi)*sin(lam);
             -sin(phi)];

    C_ecef_ned = [n_hat e_hat d_hat];

    % B en ECEF (nT)
    B_ecef = C_ecef_ned * B_ned(:,k);

    % Construir C_ecef_eci con 3 vectores base usando eci2ecef:
    % Si rotamos los ejes unitarios ECI a ECEF, obtenemos columnas de C_ecef_eci.
    ex_eci = [1;0;0];
    ey_eci = [0;1;0];
    ez_eci = [0;0;1];

    ex_ecef = eci2ecef(time_array_utc(k,:), ex_eci);  % 3x1
    ey_ecef = eci2ecef(time_array_utc(k,:), ey_eci);
    ez_ecef = eci2ecef(time_array_utc(k,:), ez_eci);

    C_ecef_eci = [ex_ecef, ey_ecef, ez_ecef];  % (ECEF <- ECI)
    C_eci_ecef = C_ecef_eci.';                 % (ECI <- ECEF)

    % B en ECI (nT)
    B_eci(:,k) = C_eci_ecef * B_ecef;
end

% Guardar para la simulación
env.B_inertial = (B_eci.') * 1e-9;   % Nx3 en Tesla (ECI)
env.dip = dip;                       % Nx1 en grados (según wrldmagm)

disp('Pre-cálculo completado.');

%% 6) SIMULACIÓN (ODE45) + WAITBAR OUTPUTFCN
disp('Iniciando Simulación...');
settings.hWaitbar = waitbar(0, 'Progress: 0%','Name', 'LINKU-SAT A Simulation Progress');

opts = odeset( ...
    'InitialStep', 1e-4, ...
    'RelTol', 1e-6, ...
    'OutputFcn', @(t,y,flag) odeWaitbar(t,y,flag,settings) ...
);

tic
[tout, x] = ode45(@(t,x) satellite_detumbling(t, x, sat, disturbance, sensors, settings, env), ...
                  [0 settings.t_final], settings.X0, opts);
elapsedTime = toc;

if isa(settings.hWaitbar,'handle') && isvalid(settings.hWaitbar)
    close(settings.hWaitbar);
end

fprintf('Simulación completada en %.2f segundos.\n', elapsedTime);

%% 7) POST-PROCESO (RECONSTRUCCIÓN CON LÓGICA DE ESTADOS)
disp('Reconstructing variables with state synchronization...');
len_out = length(tout);

% Pre-allocation
B_body_meas    = zeros(3, len_out);
Torque_ctrl    = zeros(3, len_out);
Mag_moment     = zeros(3, len_out);
Currents       = zeros(3, len_out);
Power_inst     = zeros(1, len_out);
state_rec      = zeros(1, len_out);
pointing_error = zeros(1, len_out);
T_dist_rec     = zeros(3, len_out);
K_gain_rec     = zeros(1, len_out);

% Logic variables
state_ant_post = 0;
t_event_post = -1;
t_event_first = NaN; 

% Variables para reconstrucción del PID
integral_error_post = [0; 0; 0]; % Acumulador para post-proceso

for i = 1:len_out
    t_curr = tout(i);
    q_curr = x(i, 1:4)'; 
    w_curr = x(i, 5:7)'; 
    
    % --- Onboard Computer State Logic ---
    if state_ant_post == 0 && norm(w_curr) < deg2rad(1)
        state_curr = 1;
        t_event_post = t_curr;
        if isnan(t_event_first), t_event_first = t_curr; end
    elseif state_ant_post == 1 && (t_curr - t_event_post >= 3600)
        state_curr = 2;
    else
        state_curr = state_ant_post;
    end
    state_rec(i) = state_curr;
    state_ant_post = state_curr;

    % --- Environment & Sensor Reconstruction ---
    B_in_ref = interp1(env.time, env.B_inertial, t_curr, 'linear', 'extrap')'; 
    dip_ref  = interp1(env.time, env.dip, t_curr, 'linear', 'extrap');
    B_body_true = quatRotation(quatconj(q_curr'), B_in_ref);
    B_body_meas(:,i) = mag_model(B_body_true, sensors.mag.desvEst, sensors.mag.res);
    T_dist_rec(:,i) = disturbance(t_curr);

    % --- Control Reconstruction based on State ---
    if state_curr == 0 || state_curr == 1
        % B-dot Control for Detumbling and Settling
        min_inertia = min(diag(sat.Is));
        omega_orbit = 2*pi/settings.orbit_period;
        k_val = 2 * omega_orbit * (1 + sin(deg2rad(dip_ref))) * min_inertia * 8e9;
        K_gain_rec(i) = k_val;
        [T_applied, M_applied] = detumblingControl([q_curr; w_curr], k_val, B_body_meas(:,i));
    else
        % PD Control for Nominal Alignment (State 2)
        v_vel_eci = interp1(env.time, env.Vel_inertial', t_curr, 'linear', 'extrap')';
        v_vel_body = quatRotation(quatconj(q_curr'), v_vel_eci/norm(v_vel_eci));
        pointing_error(i) = rad2deg(acos(dot([0;0;1], v_vel_body)));
        
        error_vec = cross([0;0;1], v_vel_body);
        dot_product = dot([0;0;1], v_vel_body);
        if dot_product < -0.99 && norm(error_vec) < 1e-4
            % Inyectamos un pequeño error ficticio en X para forzar el giro
            error_vec = [0.1; 0; 0]; 
        end
        % Reconstrucción del Integrador (dt variable)
        if i > 1
            dt = tout(i) - tout(i-1);
        else
            dt = 0;
        end
        integral_error_post = integral_error_post + error_vec * dt;
        
        % Anti-windup (mismo límite que en simulación)
        limit_int = 50;
        integral_error_post = max(min(integral_error_post, limit_int), -limit_int);
        
        % 5. GANANCIAS PID RELAJADAS
        % Kp: Lo suficiente para orientar el satélite sin brusquedad
        Kp = 8e-5; 
        % Ki: Muy bajo, para eliminar el error de 1.3° en el transcurso de una órbita
        Ki = 5e-7; 
        % Kd: Amortiguamiento suave
        Kd = 3e-3;
        
        % Torque Objetivo (PID)
        T_applied = (Kp * error_vec) + (Ki * integral_error_post) - (Kd * w_curr);
        
    end

    % --- Store Power & Actuator data ---
    Torque_ctrl(:,i) = T_applied;
    Mag_moment(:,i)  = M_applied;
    curr_vec = M_applied / (actuators.magnetorquer.n * actuators.magnetorquer.A);
    Currents(:,i) = curr_vec;
    Power_inst(i) = sum(abs(curr_vec)) * actuators.magnetorquer.voltage;
end

%% 8) ENHANCED PLOTS
% Interpolamos los datos pre-calculados del escenario SGP4 al vector de tiempo de la ODE
xyzout = interp1(env.time, env.Pos_inertial', tout, 'linear', 'extrap'); % Posición [Nx3]
xyzout_dot = interp1(env.time, env.Vel_inertial', tout, 'linear', 'extrap'); % Velocidad [Nx3]

% Definimos el vector de tiempo en horas para los ejes X
t_hours = tout / 3600; 

% Recuperamos los cuaterniones y velocidades angulares del vector de estado x
q0123out = x(:, 1:4); % Cuaterniones [Nx4]
pqrout   = x(:, 5:7); % Velocidades angulares [Nx3]

% Plot 7: Posición y Velocidad Orbital (ECI)
figure('Name','Orbital States (Position & Velocity)','Color','w');
subplot(2,1,1)
    plot(t_hours, xyzout(:,1)/1E3, 'b-','LineWidth',1.5); hold on; grid on;
    plot(t_hours, xyzout(:,2)/1E3, 'r-','LineWidth',1.5);
    plot(t_hours, xyzout(:,3)/1E3, 'g-','LineWidth',1.5);
    ylabel('Position (km)');
    legend('X_{ECI}', 'Y_{ECI}', 'Z_{ECI}', 'Location','eastoutside');
    title('Satellite Position in Inertial Frame');

subplot(2,1,2)
    plot(t_hours, xyzout_dot(:,1)/1E3, 'b-','LineWidth',1.5); hold on; grid on;
    plot(t_hours, xyzout_dot(:,2)/1E3, 'r-','LineWidth',1.5);
    plot(t_hours, xyzout_dot(:,3)/1E3, 'g-','LineWidth',1.5);
    xlabel('Time (h)'); ylabel('Velocity (km/s)');
    legend('V_x', 'V_y', 'V_z', 'Location','eastoutside');
    title('Satellite Orbital Velocity');
linkaxes(findall(gcf,'type','axes'),'x');

% Plot 8: Cuaterniones de Actitud (ECI to Body)
figure('Name', 'Attitude Quaternions', 'Color', 'w');
hold on; grid on;
h1 = plot(t_hours, q0123out(:,1), 'k-', 'LineWidth', 1.5); % Escalar qw
h2 = plot(t_hours, q0123out(:,2), 'r-', 'LineWidth', 1.2); % qx
h3 = plot(t_hours, q0123out(:,3), 'g-', 'LineWidth', 1.2); % qy
h4 = plot(t_hours, q0123out(:,4), 'b-', 'LineWidth', 1.2); % qz

% Añadir marcadores de transición de estado
if ~isnan(t_event_first)
    xline(t_event_first/3600, '--m', 'Settling', 'LabelVerticalAlignment', 'top','LineWidth',5);
    xline((t_event_first + 3600)/3600, '--c', 'Alignment', 'LabelVerticalAlignment', 'top','LineWidth',5);
end

title('Evolution of Attitude Quaternions (Scalar-First)');
xlabel('Time (h)'); ylabel('Normalized Value');
legend([h1, h2, h3, h4], 'q_0 (w)', 'q_1 (x)', 'q_2 (y)', 'q_3 (z)', 'Location', 'eastoutside');
ylim([-1.1 1.1]);

% Figure 1: Angular Velocity with State Markers
figure('Name','Angular Velocity Analysis','Color','w');
ax1 = subplot(2,1,1);
hold on; grid on;
plot(t_hours, rad2deg(x(:,5:7)), 'LineWidth', 1.2);
if ~isnan(t_event_first)
    xline(t_event_first/3600, '--m', 'Settling Start', 'LabelVerticalAlignment', 'bottom','LineWidth',5);
    xline((t_event_first + 3600)/3600, '--c', 'Alignment Start', 'LabelVerticalAlignment', 'bottom','LineWidth',5);
end
ylabel('\omega (deg/s)'); title('Angular Rate per Axis');
legend('\omega_x','\omega_y','\omega_z');

ax2 = subplot(2,1,2);
hold on; grid on;
plot(t_hours, rad2deg(vecnorm(x(:,5:7),2,2)), 'k', 'LineWidth', 1.5);
yline(1.0, 'r--', 'Requirement (1 deg/s)');
ylabel('|\omega| (deg/s)'); title('Angular Rate Magnitude');
xlabel('Time (h)');
linkaxes([ax1, ax2], 'x');

% Figure 2: Power and Actuation
figure('Name','Actuator Performance','Color','w');
subplot(3,1,1);
plot(t_hours, Mag_moment', 'LineWidth', 1.2); grid on;
ylabel('Moment (Am^2)'); title('Dipole Moment (Magnetorquers)');
legend('m_x','m_y','m_z');

subplot(3,1,2);
plot(t_hours, Power_inst, 'r', 'LineWidth', 1.2); grid on;
ylabel('Power (W)'); title('Instantaneous Power Consumption');

subplot(3,1,3);
plot(t_hours, pointing_error, 'g', 'LineWidth', 1.2); grid on;
ylabel('Error (deg)'); title('Z-Axis to V-bar Pointing Error (State 2)');
xlabel('Time (h)');

% Figure 3: Environment Disturbance
figure('Name','Environmental Disturbances','Color','w');
plot(t_hours, T_dist_rec'*1e3, 'LineWidth', 1.1); grid on;
ylabel('Torque (mNm)'); xlabel('Time (h)');
title('External Disturbances (Gravity Gradient, etc.)');
legend('T_x','T_y','T_z');

% Figure 4: Torque de Control Reconstruido
figure('Name','Control Torque Analysis','Color','w');
subplot(2,1,1)
    % Plot del torque por ejes (X, Y, Z) en mNm para mejor escala
    plot(t_hours, Torque_ctrl(1,:)*1e3, 'b', 'LineWidth', 1.2); hold on; grid on;
    plot(t_hours, Torque_ctrl(2,:)*1e3, 'r', 'LineWidth', 1.2);
    plot(t_hours, Torque_ctrl(3,:)*1e3, 'g', 'LineWidth', 1.2);
    
    % Marcadores de transición de estado
    if ~isnan(t_event_first)
        xline(t_event_first/3600, '--m', 'Settling', 'LabelVerticalAlignment', 'bottom');
        xline((t_event_first + 3600)/3600, '--c', 'Alignment', 'LabelVerticalAlignment', 'bottom');
    end
    
    ylabel('Torque (mN.m)');
    legend('T_x', 'T_y', 'T_z', 'Location', 'eastoutside');
    title('Control Torque per Axis (Body Frame)');

subplot(2,1,2)
    % Magnitud total del torque aplicado
    torque_magnitude = vecnorm(Torque_ctrl, 2, 1);
    plot(t_hours, torque_magnitude*1e3, 'k', 'LineWidth', 1.5); grid on;
    
    if ~isnan(t_event_first)
        xline(t_event_first/3600, '--m');
        xline((t_event_first + 3600)/3600, '--c');
    end
    
    xlabel('Time (h)'); ylabel('|T| (mN.m)');
    title('Control Torque Magnitude');
linkaxes(findall(gcf,'type','axes'),'x');

%% ==================== FUNCIONES LOCALES ====================

function x_dot = satellite_detumbling(t, x, sat, dist, sensors, settings, env)
% SATELLITE_DETUMBLING  Dinámica rígida + control magnético PID
    
    % --- Variables Persistentes para Lógica de Estados ---
    % NOTA: state_ant se mantiene aquí temporalmente. Para eliminarlo por completo, 
    % se debería usar la función "Events" de ode45 para pausar y reiniciar la simulación.
    persistent state_ant 
    
    if isempty(state_ant)
        state_ant = 0; % Estado inicial: Detumbling
    end
    
    integral_error = x(8:10); % Recuperamos la integral desde el solver
    d_integral = [0; 0; 0];   % Inicializamos la derivada de la integral
    
    % 1) Interpolación del entorno
    B_inertial_ref = interp1(env.time, env.B_inertial, t, 'linear', 'extrap')'; 
    dip_ref        = interp1(env.time, env.dip,        t, 'linear', 'extrap');
    
    % 2) Rotación inercial -> body
    q = x(1:4);
    B_body_true = quatRotation(quatconj(q'), B_inertial_ref); 
    
    % 4) Ganancia K para B-dot
    min_inertia = min(diag(sat.Is));
    omega_orbit = 2*pi/settings.orbit_period;
    
    % 3) Computadora de abordo (Determina el Estado)
    state = onboardComputer(t, state_ant, x);
    
    % Sensor (ruido + cuantización)
    B_body_meas = mag_model(B_body_true, sensors.mag.desvEst, sensors.mag.res); 

    % --- LÓGICA DE CONTROL ---
    if state == 0 || state == 1 
        % >> Estado 0/1: B-Dot (Detumbling)
        % Mantenemos la derivada de la integral en 0
        d_integral = [0; 0; 0]; 
        
        k = 2 * omega_orbit * (1 + sin(deg2rad(dip_ref))) * min_inertia * 8e9;
        [T_control, ~] = detumblingControl(x, k, B_body_meas); 
    
    else
        % >> Estado 2: Alineación Nominal (PID)
        try
            % A. Vectores de Referencia
            v_desired_eci = interp1(env.time, env.Vel_inertial', t, 'linear', 'extrap')';
            v_desired_eci = v_desired_eci/norm(v_desired_eci);
            
            q_inv = quatconj(q');
            v_target_body = quatRotation(q_inv, v_desired_eci);
            z_body = [0; 0; 1];
            
            % B. Cálculo del Error Proporcional (Vector u)
            u = cross(z_body, v_target_body); % Dirección y magnitud del error (sen(theta))
            
            % C. Corrección de Singularidad (180 grados)
            dot_product = dot(z_body, v_target_body);
            if dot_product < -0.99 && norm(u) < 1e-4
                u = [0.1; 0; 0]; % Empujón para salir del equilibrio inestable
            end
            
            % D. Pasamos la derivada al vector de estado para que ode45 la integre
            % Matemáticamente, la derivada del error integral es el error mismo (u)
            d_integral = u;
            
            % Anti-Windup: Limitar el error integral para que no crezca infinito
            limit_int = 50; 
            integral_error = max(min(integral_error, limit_int), -limit_int);

            % E. Ganancias PID (Sintonización Sugerida)
            % Ganancias PID Actualizadas
            % Kp: Lo suficiente para orientar el satélite sin brusquedad
            Kp = 8e-5; 
            % Ki: Muy bajo, para eliminar el error de 1.3° en el transcurso de una órbita
            Ki = 5e-7; 
            % Kd: Amortiguamiento suave
            Kd = 3e-3;
            
            % F. Ley de Control PID
            % T = P + I - D
            T_control = (Kp * u) + (Ki * integral_error) - (Kd * x(5:7)); 
            
        catch
            T_control = [0;0;0];
        end 
    end
    
    % 6) Disturbios
    T_dist = dist(t); T_dist = T_dist(:);
    
    % 7) Torque total
    T_total = T_dist + T_control;
    
    % 8) Dinamicas del satelite
    x_dot_dyn = satellite_dynamics(x, sat, T_total);
    x_dot = [x_dot_dyn; d_integral]; % Vector de salida 10x1
    
    % 9) Update state
    state_ant = state;
end

function [Tc, muB_sat] = detumblingControl(state, k, B_body)
% DETUMBLINGCONTROL  Ley tipo B-dot: mu = k (w x B), Tc = mu x B
    omega = state(5:7); omega = omega(:);
    B_body = B_body(:);

    muB = k * cross(omega, B_body);

    % Saturación por dipolo máximo (0.2 A m^2)
    muB_sat = max(min(muB, 0.2), -0.2);

    Tc = cross(muB_sat, B_body);
end

function mag_bm = mag_model(mag_b, sig_tam, sig_res)
% MAG_MODEL  Magnetómetro: ruido blanco + cuantización
% Entrada/salida en 3x1
    mag_b = mag_b(:);
    mag_b_noise = mag_b + sig_tam*randn(3,1);
    mag_bm = round(mag_b_noise./sig_res).*sig_res;
end

function rotX = quatRotation(q, x)
% QUATROTATION  Rota vector x (3x1) usando cuaternión q (1x4)
    x = x(:);
    qx = [0, x(1), x(2), x(3)];
    if size(q,1)==4 && size(q,2)==1
        q = q';
    end
    qrotX = quatmultiply(quatmultiply(q, qx), quatconj(q));
    rotX = qrotX(2:4).';
    rotX = rotX(:);
end

function stop = odeWaitbar(t,~,flag,settings)
% ODEWAITBAR  OutputFcn para actualizar waitbar SOLO 5 veces (0-25-50-75-100%)

stop = false;

if ~isa(settings.hWaitbar,'handle') || ~isvalid(settings.hWaitbar)
    return;
end

% Guardamos el "siguiente umbral" a mostrar (0..4)
persistent nextStepShown
if isempty(nextStepShown)
    nextStepShown = 0;
end

switch flag
    case 'init'
        nextStepShown = 0;
        waitbar(0, settings.hWaitbar, 'Progress: 0%');

    case ''
        % Progreso actual en [0,1]
        progress = min(max(t(end) / settings.t_final, 0), 1);

        % Queremos 5 updates: step = 0..4
        % thresholds: 0, 0.25, 0.50, 0.75, 1.00
        stepNow = floor(progress * 4 + 1e-12);  % 0..4

        if stepNow >= nextStepShown
            % Actualiza SOLO cuando cruza el siguiente umbral
            pct = 25 * stepNow;  % 0,25,50,75,100
            waitbar(stepNow/4, settings.hWaitbar, sprintf('Progress: %d%%', pct));
            nextStepShown = stepNow + 1;
        end

    case 'done'
        waitbar(1, settings.hWaitbar, 'Progress: 100%');
        nextStepShown = 0;
end
end


function [tiempo_inicio, index] = obtainStableTime(time, signal, lowerLimit, upperLimit)
    indices = find(signal >= lowerLimit & signal <= upperLimit);
    if ~isempty(indices)
        index = indices(1);
        tiempo_inicio = time(index);
    else
        tiempo_inicio = NaN;
        index = NaN;
        disp('La señal no está dentro de la franja en ningún momento.');
    end
end

function porcentaje = percentageOfSignal(signal, lowerRange, upperRange)
    in_range = (signal >= lowerRange) & (signal <= upperRange);
    porcentaje = (sum(in_range) / length(signal)) * 100;
end

function potencia_media_rectangular = potenciaMedia(t, intensity)
% Integra i(t)^2 con regla rectangular (útil si luego multiplicas por R)
    t = t(:); intensity = intensity(:);
    dt = diff(t);
    potencia_media_rectangular = sum(intensity(1:end-1).^2 .* dt) / (t(end) - t(1));
end

function dy = decimale_vector(date_matrix)
% date_matrix: Nx6 (datevec). Devuelve decimal year
    years = date_matrix(:,1);
    is_leap = (mod(years,4)==0 & mod(years,100)~=0) | (mod(years,400)==0);
    days_in_year = 365 + is_leap;

    start_of_years = datenum([years, ones(length(years),2), zeros(length(years),3)]);
    current_dates  = datenum(date_matrix);

    dy = years + (current_dates - start_of_years) ./ days_in_year;
end

function x_dot = satellite_dynamics(x, sat, T_total)
    q = x(1:4);
    w = x(5:7);

    Xi = [-q(2) -q(3) -q(4);
           q(1) -q(4)  q(3);
           q(4)  q(1) -q(2);
          -q(3)  q(2)  q(1)];
    q_dot = 0.5 * Xi * w;

    w_dot = sat.Is \ (T_total - cross(w, sat.Is*w));

    x_dot = [q_dot; w_dot];
end

function state = onboardComputer(t, state_ant, x)
    w = x(5:7);
    threshold = deg2rad(1); % Umbral de 1 deg/s
    
    % Variables que sobreviven entre llamadas de ode45
    persistent t_event
    
    % Inicialización de t_event si está vacío
    if isempty(t_event)
        t_event = -1; 
    end

    % LÓGICA DE TRANSICIÓN
    if state_ant == 0 && norm(w) < threshold
        % HIT: Se alcanzó la velocidad por primera vez
        state = 1;
        t_event = t; % Guardamos el tiempo exacto del evento
        fprintf('At t=%.2f s: Target reached. State 0 -> 1. Timer started.\n', t);
        
    elseif state_ant == 1
        % Estamos en el estado 1, verificamos si ya pasó 1 hora (3600 s)
        tiempo_transcurrido = t - t_event;
        
        if tiempo_transcurrido >= 3600
            state = 2; % Cambiar a Estado 2 (ej. Apuntamiento Nominal)
            fprintf('At t=%.2f s: 1 hour passed in State 1. Switching to State 2.\n', t);
        else
            state = 1; % Seguir en Estado 1
        end
        
    else
        state = state_ant; % Mantener el estado actual (Estado 2 u otros)
    end
end