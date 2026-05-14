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

%% Actuadores: Magnetorquers (iADCS400 Model)
actuators.magnetorquer.power            = 600e-3;       % W (Típicamente ~0.6W por eje a 5V)
actuators.magnetorquer.MaxPower         = 600e-3;       % W
actuators.magnetorquer.voltage          = 5;            % V
actuators.magnetorquer.dimensions       = [80,10,10];   % mm (l,w,h) - Longitud del núcleo
actuators.magnetorquer.nominalDipole    = 0.4;          % A m^2 (0.4 Am^2 es estándar en serie 400)
actuators.magnetorquer.maxNominalDipole = 0.4;          % A m^2

% Área (m^2) usando mm -> m: A = (h*l) [mm^2] * 1e-6
actuators.magnetorquer.A  = actuators.magnetorquer.dimensions(2) * actuators.magnetorquer.dimensions(3) * 1e-6;
% Estimación de número de vueltas "equivalente" (ojo: modelo simplificado)
actuators.magnetorquer.n  = actuators.magnetorquer.nominalDipole * actuators.magnetorquer.voltage / ...
                            (actuators.magnetorquer.A * actuators.magnetorquer.power);
actuators.magnetorquer.maxCurrent = actuators.magnetorquer.maxNominalDipole / actuators.magnetorquer.voltage;

% Parámetros del Núcleo (Histéresis Simplificada)
actuators.magnetorquer.residualDipole = [0.004; -0.002; 0.003]; % ~1% del dipolo máximo típico
actuators.magnetorquer.linearityFactor = 2.0; % Factor de curvatura suave B-H (tanh)

%% Sensores: Magnetómetro
sensors.mag.desvEst = 150.00e-9; % T (ruido MEMS comercial típico)
sensors.mag.res     = 10.00e-9;  % T (resolución mejorada)
sensors.mag.normDesvEst = 0.5;
sensors.mag.tau     = 0.5;      % Constante de tiempo del filtro Pasa-Bajo [s]

%% Sensores: Giroscopio (IMU Model)
sensors.gyro.scaleFactor = [0.005; 0.005; 0.005]; % 0.5% error de escala
sensors.gyro.misalign = [0, 0.001, 0.001; 0.001, 0, 0.001; 0.001, 0.001, 0]; % m_ij [rad]
sensors.gyro.noiseStd = deg2rad(0.05); % Ruido blanco eta_g [rad/s] mejorado
sensors.gyro.biasWalkStd = deg2rad(0.005); % Inestabilidad del bias sigma_RW [rad/s / sqrt(s)]
sensors.gyro.tau = 0.1; % Constante de tiempo del filtro Pasa-Bajo [s]
sensors.gyro.yMin = deg2rad(-250); % Límite de saturación inferior
sensors.gyro.yMax = deg2rad(250);  % Límite de saturación superior
sensors.gyro.bits = 16; % n_bits del ADC
sensors.gyro.Vref = 3.3; % Voltaje de referencia del ADC

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

%% 3.1) CONFIGURACIÓN DE CONTROL (PID & B-DOT)
ctrl.Kp = 8e-5;
ctrl.Ki = 5e-5; % Incrementado 100x para vencer la remanencia del núcleo y perturbaciones
ctrl.Kd = 3e-3;
ctrl.limit_int = 50;
ctrl.bang_bang_factor = 1e3; % Factor multiplicador para forzar frenado agresivo

% Ganancia teórica para el controlador B-Dot (Cross-Product)
% K_bdot = [4*pi*(1+sin(i))*I_min] / [T_orb * B_avg^2]
B_avg = 40e-6; % Campo magnético promedio en LEO (~40,000 nT convertido a Tesla)
ctrl.k_bdot = (4 * pi / orbit.period) * (1 + sin(deg2rad(orbit.inclination))) * min(diag(sat.Is)) / (B_avg^2);

%% 4) CONFIGURACIÓN DE SIMULACIÓN
settings.startTime = datetime(2025,10,10,23,30,00);
settings.startTime.TimeZone = "UTC";

settings.number_of_orbits = 10; % 10 órbitas para dar margen físico de frenado
settings.sample_step = 100;
settings.t_final = orbit.period * settings.number_of_orbits;
settings.orbit_period = orbit.period;                 

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

%% 6) MONTE CARLO & SIMULACIÓN (ODE45)
num_mc_runs = 5; % Cambiar a 1 para simulación única, >1 para Monte Carlo

mc_detumble_times = NaN(num_mc_runs, 1);
mc_energy = NaN(num_mc_runs, 1);
mc_omega_hist = cell(num_mc_runs, 1);
mc_time_hist = cell(num_mc_runs, 1);
nominal_Is = sat.Is;

disp('Iniciando Simulaciones...');

for mc_iter = 1:num_mc_runs
    fprintf('\n--- Iteración Monte Carlo %d / %d ---\n', mc_iter, num_mc_runs);
    
    % 1. Aleatorizar Condiciones (Tasas de eyección realistas: entre -5 y +5 deg/s)
    initial.omega.omega0_x = deg2rad(10 * rand() - 5);
    initial.omega.omega0_y = deg2rad(10 * rand() - 5);
    initial.omega.omega0_z = deg2rad(10 * rand() - 5);
    
    % 2. Aleatorizar Tensor de Inercia (+/- 10% de variación gaussiana)
    err_Is = 0.10 * randn(3,3);
    err_Is = (err_Is + err_Is')/2;
    sat.Is = nominal_Is .* (1 + err_Is);
    
    % 3. Actualizar ganancia y Vector de Estado inicial
    ctrl.k_bdot = (4 * pi / orbit.period) * (1 + sin(deg2rad(orbit.inclination))) * min(diag(sat.Is)) / (B_avg^2);
    settings.X0 = [initial.attitude.q0123_0; initial.omega.omega0_x; initial.omega.omega0_y; initial.omega.omega0_z; 0; 0; 0];
    
    % 4. Ejecutar Integrador
    if num_mc_runs == 1
        settings.hWaitbar = waitbar(0, 'Progress: 0%','Name', 'LINKU-SAT A Simulation Progress');
        opts = odeset('InitialStep', 1e-4, 'RelTol', 1e-6, 'OutputFcn', @(t,y,flag) odeWaitbar(t,y,flag,settings));
    else
        opts = odeset('InitialStep', 1e-4, 'RelTol', 1e-6); % Sin waitbar para acelerar cálculo masivo
    end
    
    tic
    [tout, x] = ode45(@(t,x) satellite_detumbling(t, x, sat, disturbance, sensors, settings, env, ctrl, actuators), ...
                      [0 settings.t_final], settings.X0, opts);
    elapsedTime = toc;
    fprintf('Simulación completada en %.2f segundos.\n', elapsedTime);
    
    if num_mc_runs == 1 && isa(settings.hWaitbar,'handle') && isvalid(settings.hWaitbar)
        close(settings.hWaitbar);
    end
    
    mc_time_hist{mc_iter} = tout / 3600;
    mc_omega_hist{mc_iter} = rad2deg(vecnorm(x(:,5:7), 2, 2));

    %% 7) POST-PROCESO DE ESTA ITERACIÓN
    len_out = length(tout);

% Pre-allocation
B_body_meas    = zeros(3, len_out);
B_body_true_rec= zeros(3, len_out); % Array para almacenar el campo verdadero
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

% Variables IMU post-proceso
bias_g_post = [0;0;0];
w_filt_post = x(1, 5:7)';
mag_noise_post = [0;0;0];
B_in_ref0 = interp1(env.time, env.B_inertial, tout(1), 'linear', 'extrap')';
B_filt_post = quatRotation(quatconj(x(1, 1:4)), B_in_ref0);

for i = 1:len_out
    t_curr = tout(i);
    q_curr = x(i, 1:4)'; 
    w_curr = x(i, 5:7)'; 
    int_err_curr = x(i, 8:10)'; % Recuperamos el error integrado por ode45
    
    % --- Onboard Computer State Logic ---
    if state_ant_post == 0 && norm(w_curr) < deg2rad(1)
        state_curr = 1;
        t_event_post = t_curr;
        if isnan(t_event_first), t_event_first = t_curr; end
    else
        state_curr = state_ant_post;
    end
    state_rec(i) = state_curr;
    state_ant_post = state_curr;

    % --- Environment & Sensor Reconstruction ---
    B_in_ref = interp1(env.time, env.B_inertial, t_curr, 'linear', 'extrap')'; 
    dip_ref  = interp1(env.time, env.dip, t_curr, 'linear', 'extrap');
    B_body_true = quatRotation(quatconj(q_curr'), B_in_ref);
    B_body_true_rec(:,i) = B_body_true; % Guardamos el campo real para graficar
    T_dist_rec(:,i) = disturbance(t_curr);

    % --- Consistent IMU Reconstruction ---
    dt = 0;
    if i > 1
        dt = tout(i) - tout(i-1);
    end
    
    [B_body_meas(:,i), mag_noise_post, B_filt_post] = imu_mag_model(B_body_true, dt, mag_noise_post, B_filt_post, sensors);
    [w_meas, bias_g_post, w_filt_post] = imu_gyro_model(w_curr, dt, bias_g_post, w_filt_post, sensors);
    state_meas = [q_curr; w_meas; int_err_curr];

    % --- Control Reconstruction based on State ---
    if state_curr == 0 || state_curr == 1
        % B-dot Puro (Sin giroscopio) + Bang-Bang
        k_val = ctrl.k_bdot * ctrl.bang_bang_factor;
        K_gain_rec(i) = k_val;
        
        % Derivada continua (analítica) para B-Dot: B_dot = -w x B
        % Es matemáticamente estable y evita ruido numérico
        B_dot_post = -cross(w_meas, B_body_meas(:,i));
        
        [T_applied, M_applied] = detumblingControl_PureBDot(B_dot_post, k_val, B_body_meas(:,i), actuators);
        [~, curr_vec] = magnetorquer_model(M_applied, actuators); % Extraer corriente real consumida
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
        
        % Reconstruimos T_pid deseado
        int_err_sat = max(min(int_err_curr, ctrl.limit_int), -ctrl.limit_int);
        T_pid_desired = (ctrl.Kp * error_vec) + (ctrl.Ki * int_err_sat) - (ctrl.Kd * w_meas);
        
        % Reconstruimos el Cross-Product Steering
        B_meas = B_body_meas(:,i);
        B_norm_sq = norm(B_meas)^2;
        if B_norm_sq > 1e-15
            M_cmd = cross(B_meas, T_pid_desired) / B_norm_sq;
        else
            M_cmd = [0;0;0];
        end
        
        % Compensación de Software: Restamos el dipolo residual esperado para anularlo
        M_cmd = M_cmd - actuators.magnetorquer.residualDipole;
        
        % Pasamos comando por el Modelo de Magnetorquer
        [M_applied, curr_vec] = magnetorquer_model(M_cmd, actuators);
        T_applied = cross(M_applied, B_meas);
        
    end

    % --- Store Power & Actuator data ---
    Torque_ctrl(:,i) = T_applied;
    Mag_moment(:,i)  = M_applied;
    Currents(:,i) = curr_vec;
    Power_inst(i) = sum(abs(curr_vec)) * actuators.magnetorquer.voltage;
end
    
    % Guardar métricas Monte Carlo
    mc_detumble_times(mc_iter) = t_event_first / 3600; % horas
    mc_energy(mc_iter) = trapz(tout, Power_inst); % Joules
    fprintf('Detumbling: %.2f horas | Energía: %.2f J\n', mc_detumble_times(mc_iter), mc_energy(mc_iter));
    
end % Fin del bucle Monte Carlo

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

% Figure 5: Magnetic Field: True vs Measured
figure('Name','Magnetic Field: True vs Measured','Color','w');

subplot(3,1,1);
plot(t_hours, B_body_true_rec(1,:)*1e9, 'k', 'LineWidth', 1.5); hold on; grid on;
plot(t_hours, B_body_meas(1,:)*1e9, 'r--', 'LineWidth', 1.0);
ylabel('B_x (nT)'); title('Magnetic Field (Body Frame) - True vs Measured');
legend('True Model', 'Measured (Sensor)', 'Location', 'eastoutside');

subplot(3,1,2);
plot(t_hours, B_body_true_rec(2,:)*1e9, 'k', 'LineWidth', 1.5); hold on; grid on;
plot(t_hours, B_body_meas(2,:)*1e9, 'g--', 'LineWidth', 1.0);
ylabel('B_y (nT)');
legend('True Model', 'Measured (Sensor)', 'Location', 'eastoutside');

subplot(3,1,3);
plot(t_hours, B_body_true_rec(3,:)*1e9, 'k', 'LineWidth', 1.5); hold on; grid on;
plot(t_hours, B_body_meas(3,:)*1e9, 'b--', 'LineWidth', 1.0);
ylabel('B_z (nT)'); xlabel('Time (h)');
legend('True Model', 'Measured (Sensor)', 'Location', 'eastoutside');

linkaxes(findall(gcf,'type','axes'),'x');

% --- MONTE CARLO PLOTS ---
if num_mc_runs > 1
    figure('Name','Monte Carlo: Comprehensive Performance Analysis','Color','w', 'Position', [100, 100, 1200, 800]);
    
    % Función anónima para calcular la curva Normal (Gaussiana)
    gaussian_pdf = @(x, mu, sig) (1 ./ (sig * sqrt(2*pi))) .* exp(-0.5 * ((x - mu)./sig).^2);
    
    % 1. Distribución de Tiempos de Detumbling
    subplot(2,2,1);
    valid_times = mc_detumble_times(~isnan(mc_detumble_times));
    if ~isempty(valid_times)
        mu_t = mean(valid_times); sig_t = std(valid_times);
        histogram(valid_times, 15, 'Normalization', 'pdf', 'FaceColor', '#0072BD', 'EdgeColor', 'w'); hold on;
        x_t = linspace(min(valid_times)-sig_t, max(valid_times)+sig_t, 100);
        plot(x_t, gaussian_pdf(x_t, mu_t, sig_t), 'r-', 'LineWidth', 2);
        title(sprintf('Detumbling Time Distribution (\\mu = %.2fh, \\sigma = %.2fh)', mu_t, sig_t));
        xlabel('Time (Hours)'); ylabel('Probability Density'); grid on;
    end
    
    % 2. Distribución de Consumo Energético
    subplot(2,2,2);
    valid_energy = mc_energy(~isnan(mc_energy));
    if ~isempty(valid_energy)
        mu_e = mean(valid_energy); sig_e = std(valid_energy);
        histogram(valid_energy, 15, 'Normalization', 'pdf', 'FaceColor', '#D95319', 'EdgeColor', 'w'); hold on;
        x_e = linspace(min(valid_energy)-sig_e, max(valid_energy)+sig_e, 100);
        plot(x_e, gaussian_pdf(x_e, mu_e, sig_e), 'k-', 'LineWidth', 2);
        title(sprintf('Energy Consumed Distribution (\\mu = %.0f J, \\sigma = %.0f J)', mu_e, sig_e));
        xlabel('Total Energy (Joules)'); ylabel('Probability Density'); grid on;
    end
    
    % 3. Evolución de Magnitud Angular de Todas las Simulaciones
    subplot(2,2,[3 4]);
    hold on; grid on;
    for k = 1:num_mc_runs
        plot(mc_time_hist{k}, mc_omega_hist{k}, 'LineWidth', 0.8, 'Color', [0.5 0.5 0.5 0.6]);
    end
    yline(1.0, 'r--', 'Requirement (1 deg/s)', 'LineWidth', 2);
    title('Angular Velocity Magnitude Overview (All Monte Carlo Iterations)');
    xlabel('Time (Hours)'); ylabel('|\omega| (deg/s)');
end

%% ==================== FUNCIONES LOCALES ====================

function x_dot = satellite_detumbling(t, x, sat, dist, sensors, settings, env, ctrl, actuators)
% SATELLITE_DETUMBLING  Dinámica rígida + control magnético PID
    
    % --- Variables Persistentes para Lógica de Estados e IMU ---
    persistent state_ant t_prev bias_g w_filt mag_noise B_filt
    
    if isempty(state_ant)
        state_ant = 0; % Estado inicial: Detumbling
        t_prev = t;
        bias_g = [0;0;0]; % b_g(t=0) Bias inicial
        w_filt = x(5:7);  % Inicializamos el filtro digital para evitar picos
        mag_noise = [0;0;0];
        B_in_ref0 = interp1(env.time, env.B_inertial, t, 'linear', 'extrap')';
        B_filt = quatRotation(quatconj(x(1:4)'), B_in_ref0);
    end
    
    integral_error = x(8:10); % Variable de estado proveniente del ode45
    d_int = [0;0;0];          % Derivada inicial del integrador del PID
    
    % Calculamos dt explícito para el Random Walk y el filtro discreto de la IMU
    dt = t - t_prev;
    if dt < 0; dt = 0; end % Prevenir pasos hacia atrás de ode45
    
    % 1) Interpolación del entorno
    B_inertial_ref = interp1(env.time, env.B_inertial, t, 'linear', 'extrap')'; 
    dip_ref        = interp1(env.time, env.dip,        t, 'linear', 'extrap');
    
    % 2) Rotación inercial -> body
    q = x(1:4);
    B_body_true = quatRotation(quatconj(q'), B_inertial_ref); 
    
    % 3) Computadora de abordo (Determina el Estado)
    state = onboardComputer(t, state_ant, x);
    
    % Sensor (ruido congelado + cuantización + filtro pasa-bajo)
    [B_body_meas, mag_noise, B_filt] = imu_mag_model(B_body_true, dt, mag_noise, B_filt, sensors);

    % IMU (Medición para TODOS los estados)
    [w_meas, bias_g, w_filt] = imu_gyro_model(x(5:7), dt, bias_g, w_filt, sensors);
    state_meas = x;
    state_meas(5:7) = w_meas;

    % --- LÓGICA DE CONTROL ---
    if state == 0 || state == 1 
        % >> Estado 0/1: B-Dot Puro (Sin Giroscopio) + Bang-Bang
        
        % B_dot analítico para estabilidad extrema del integrador ode45
        B_dot = -cross(w_meas, B_body_meas);
        
        k_val = ctrl.k_bdot * ctrl.bang_bang_factor;
        [T_control, ~] = detumblingControl_PureBDot(B_dot, k_val, B_body_meas, actuators); 
    
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
            
            % D. Pasamos la derivada del error a ode45 para que lo integre
            d_int = u; 
            
            % E. Anti-Windup (Limitamos su acción)
            int_err_sat = max(min(integral_error, ctrl.limit_int), -ctrl.limit_int);
            
            % F. Ley de Control PID Ideal (Torque deseado)
            T_pid_desired = (ctrl.Kp * u) + (ctrl.Ki * int_err_sat) - (ctrl.Kd * w_meas); 
            
            % G. CROSS-PRODUCT STEERING: Traducir Torque Deseado a Dipolo Real (Magnetorquers)
            B_norm_sq = norm(B_body_meas)^2;
            if B_norm_sq > 1e-15
                M_cmd = cross(B_body_meas, T_pid_desired) / B_norm_sq;
            else
                M_cmd = [0;0;0];
            end
            
            % H. Compensación de Histéresis: Restamos el dipolo residual esperado
            M_cmd = M_cmd - actuators.magnetorquer.residualDipole;
            
            % Pasamos el comando por el Modelo Físico del Magnetorquer
            [M_sat, ~] = magnetorquer_model(M_cmd, actuators);
            
            % Torque real aplicado a la planta
            T_control = cross(M_sat, B_body_meas);
            
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
    x_dot = [x_dot_dyn; d_int]; % Añadimos la integración del PID al vector
    
    % 9) Update state
    state_ant = state;
    
    % Actualizar tiempo para la IMU si el solver avanzó
    if dt > 0
        t_prev = t;
    end
end

function [Tc, muB_sat] = detumblingControl_PureBDot(B_dot, k, B_body, actuators)
% DETUMBLINGCONTROL_PUREBDOT  Ley B-dot puro (Sin IMU): mu = -k * B_dot
    B_dot = B_dot(:);
    B_body = B_body(:);

    muB = -k * B_dot;

    % Modelo Físico de saturación del actuador
    [muB_sat, ~] = magnetorquer_model(muB, actuators);

    Tc = cross(muB_sat, B_body);
end

function [B_meas, noise_new, B_filt_new] = imu_mag_model(B_true, dt, noise_old, B_filt_old, sensors)
% IMU_MAG_MODEL Simula ruido, cuantización y filtro pasa-bajo del magnetómetro
    
    % 1. Generación de ruido (congelado en sub-pasos de ode45)
    if dt > 0
        eta_m = sensors.mag.desvEst * randn(3,1);
        noise_new = eta_m;
    else
        noise_new = noise_old;
    end
    
    B_raw = B_true(:) + noise_new;
    
    % 2. Cuantización (Resolución del sensor)
    B_quant = round(B_raw ./ sensors.mag.res) .* sensors.mag.res;
    
    % 3. Filtro Pasa-Bajo Digital
    if dt > 0
        alpha = dt / (sensors.mag.tau + dt);
        B_filt_new = (1 - alpha) * B_filt_old(:) + alpha * B_quant;
    else
        B_filt_new = B_filt_old(:); 
    end
    
    B_meas = B_filt_new;
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
        % Nos mantenemos en el Estado 1 indefinidamente (Solo Detumbling)
        state = 1; 
        
    else
        state = state_ant; % Mantener el estado actual (Estado 2 u otros)
    end
end

function [M_real, I_real] = magnetorquer_model(M_cmd, actuators)
% MAGNETORQUER_MODEL Simula el comportamiento físico del actuador electromagnético.
% Convierte el dipolo comandado a corriente requerida, aplica límites de hardware 
% (saturación de amperaje), y modela la curva B-H del núcleo ferromagnético 
% incluyendo saturación suave e histéresis estática (remanencia).

    n = actuators.magnetorquer.n;
    A = actuators.magnetorquer.A;
    I_max = actuators.magnetorquer.maxCurrent; % Límite de corriente del hardware
    M_max = actuators.magnetorquer.maxNominalDipole;
    M_rem = actuators.magnetorquer.residualDipole;
    k_lin = actuators.magnetorquer.linearityFactor;

    % 1. Corriente ideal requerida por eje (I = M / (n*A))
    I_cmd = M_cmd / (n * A);

    % 2. Saturación impuesta por la electrónica de potencia (Driver)
    I_real = max(min(I_cmd, I_max), -I_max);

    % 3. Modelo del Núcleo (Saturación suave y Remanencia Magnética)
    % La función tanh() imita la curva de saturación magnética del hierro.
    % M_rem simula el magnetismo que "retiene" el núcleo cuando I = 0.
    M_core = M_max * tanh(k_lin * (I_real / I_max)) + M_rem;
    
    % 4. Dipolo real generado (con límite físico absoluto de seguridad)
    M_real = max(min(M_core, M_max), -M_max);
end

function [w_meas, bias_new, w_filt_new] = imu_gyro_model(w_true, dt, bias_old, w_filt_old, sensors)
% IMU_GYRO_MODEL Simula los errores matemáticos de un giroscopio

    % 1. Matrices de Error (Ecuación 1)
    s = sensors.gyro.scaleFactor;
    m = sensors.gyro.misalign;
    
    T_sf = diag([1+s(1), 1+s(2), 1+s(3)]);
    T_ma = [1,      m(1,2), m(1,3);
            m(2,1), 1,      m(2,3);
            m(3,1), m(3,2), 1];
            
    % 2. Ruido e inestabilidad del Bias (Ecuaciones 2 y 3)
    eta_g = sensors.gyro.noiseStd * randn(3,1);
    
    if dt > 0
        eta_bg = sensors.gyro.biasWalkStd * sqrt(dt) * randn(3,1);
        bias_new = bias_old + eta_bg;
    else
        bias_new = bias_old; % El tiempo no ha avanzado
    end
    
    w_raw = T_ma * T_sf * w_true + bias_new + eta_g;
    
    % 3. ADC: Saturación y Cuantización Digital (Ecuaciones 7, 8 y 9)
    w_sat = max(min(w_raw, sensors.gyro.yMax), sensors.gyro.yMin);
    
    levels = 2^sensors.gyro.bits - 1;
    D_out = round( levels * (w_sat - sensors.gyro.yMin) / (sensors.gyro.yMax - sensors.gyro.yMin) );
    w_adc = sensors.gyro.yMin + ((sensors.gyro.yMax - sensors.gyro.yMin) / levels) * D_out;
    
    % 4. Filtro Pasa-Bajo Digital (Ecuación 4)
    if dt > 0
        alpha = dt / (sensors.gyro.tau + dt);
        w_filt_new = (1 - alpha) * w_filt_old + alpha * w_adc;
    else
        w_filt_new = w_filt_old; 
    end
    
    w_meas = w_filt_new;
end