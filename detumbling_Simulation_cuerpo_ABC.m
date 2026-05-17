clear; clc; close all;

%% 1) PARÁMETROS DEL SATÉLITE (LINKU - 12U)
% sat.Is = [0.103018304995791, -0.000007224537238, 0.000015479696200;
%          -0.000007224537238,  0.103187907740666, -0.000142288034562;
%           0.000015479696200, -0.000142288034562,  0.054153798020147];
% sat.mass = 6.843382000000000; % kg

% % % %% 1) PARÁMETROS DEL SATÉLITE (12U STANDAR)
% % % Ixx = 0.2785;
% % % Iyy = 0.2792; % Ligeramente diferente a Ixx por asimetría interna
% % % Izz = 0.1705;

Ixx = 0.0577;
Iyy = 0.5777; 
Izz = 0.0444;
sat.mass = 5.5; % kg

% % Productos de inercia (típicamente 1-5% de los principales)
% Ixy = -0.005; 
% Ixz =  0.003; 
% Iyz =  0.004;

Ixy =  0.001; 
Ixz =  0.001; 
Iyz =  -0.002;

sat.Is = [Ixx,  Ixy,  Ixz;
          Ixy,  Iyy,  Iyz;
          Ixz,  Iyz,  Izz];

%sat.mass = 20.0;

%% Actuadores: Magnetorquers (iADCS400 Model)
% actuators.magnetorquer.power            = 600e-3;       % W (Típicamente ~0.6W por eje a 5V)
% actuators.magnetorquer.MaxPower         = 600e-3;       % W
% actuators.magnetorquer.voltage          = 5;            % V
% actuators.magnetorquer.dimensions       = [80,10,10];   % mm (l,w,h) - Longitud del núcleo
% actuators.magnetorquer.nominalDipole    = 0.4;          % A m^2 (0.4 Am^2 es estándar en serie 400)
% actuators.magnetorquer.maxNominalDipole = 0.4;          % A m^2
actuators.magnetorquer.power            = 360e-3;       % W
actuators.magnetorquer.MaxPower         = 360e-3;       % W
actuators.magnetorquer.voltage          = 5;            % V
actuators.magnetorquer.dimensions       = [30,30,30];   % mm (w,h,l)
actuators.magnetorquer.nominalDipole    = 0.1;          % A m^2
actuators.magnetorquer.maxNominalDipole = 0.1;          % A m^2



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
ctrl.pointing.P = ctrl.Kd * eye(3); % Ganancia de error de velocidad angular para feedback quaternion
ctrl.pointing.K = ctrl.Kp * eye(3); % Ganancia de error de actitud para feedback quaternion

% Ganancia teórica para el controlador B-Dot (Cross-Product)
% K_bdot = [4*pi*(1+sin(i))*I_min] / [T_orb * B_avg^2]
B_avg = 40e-6; % Campo magnético promedio en LEO (~40,000 nT convertido a Tesla)
ctrl.k_bdot = (4 * pi / orbit.period) * (1 + sin(deg2rad(orbit.inclination))) * min(diag(sat.Is)) / (B_avg^2);

%% 4) CONFIGURACIÓN DE SIMULACIÓN
settings.startTime = datetime(2025,10,10,23,30,00);
settings.startTime.TimeZone = "UTC";

settings.number_of_orbits = 3; % 10 órbitas para dar margen físico de frenado
settings.sample_step = 100;
settings.t_final = orbit.period * settings.number_of_orbits;
settings.orbit_period = orbit.period;                 
settings.enable_alignment = true; % true: tras detumbling pasa al modo pointing/alineamiento
settings.stop_at_state1 = false; % true: detener ode45 cuando |omega| < 1 deg/s
settings.fast_state1_eval = false; % true: omite reconstruccion pesada de senales tras ode45
settings.use_parallel_mc = false; % true: usa parfor para Monte Carlo en modo fast_state1_eval
settings.initial_state = 2; % 0: detumbling normal, 2: arranque directo en pointing
settings.detumble_threshold = deg2rad(1);
settings.detumble_exit_threshold = deg2rad(1.2); % Histeresis: si sube de esto, se reinicia la verificacion
settings.state1_hold_time = 3600; % Tiempo minimo estable antes de aceptar estado 1 [s]
settings.alignment_wait_time = 600; % Tiempo de espera tras detumbling antes de alineamiento [s]
settings.save_results = true;
settings.results_dir = 'results';

%% 5) Configuracion de computadora de abordo
state_ant = settings.initial_state;

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
num_mc_runs = 1; % Pointing verification run; raise this manually for Monte Carlo.
mc_detumble_times = NaN(num_mc_runs, 1);
mc_first_cross_times = NaN(num_mc_runs, 1);
mc_energy = NaN(num_mc_runs, 1);
mc_omega_hist = cell(num_mc_runs, 1);
mc_time_hist = cell(num_mc_runs, 1);
nominal_Is = sat.Is;

disp('Iniciando Simulaciones...');

useParallelMc = settings.use_parallel_mc && settings.fast_state1_eval && num_mc_runs > 1 && ...
                license('test', 'Distrib_Computing_Toolbox');

if useParallelMc
    pool = gcp('nocreate');
    if isempty(pool)
        parpool;
    end

    sat_mc = sat;
    if isfield(sat_mc, 'satSGP4')
        sat_mc = rmfield(sat_mc, 'satSGP4');
    end

    fprintf('Ejecutando Monte Carlo en paralelo (%d corridas)...\n', num_mc_runs);
    parfor mc_iter = 1:num_mc_runs
        [mc_detumble_times(mc_iter), mc_first_cross_times(mc_iter), mc_energy(mc_iter), mc_time_hist{mc_iter}, mc_omega_hist{mc_iter}] = ...
            runFastState1McIteration(mc_iter, initial, nominal_Is, sat_mc, orbit, B_avg, env, ...
                                     disturbance, sensors, settings, ctrl, actuators);
    end
else
    if settings.use_parallel_mc && settings.fast_state1_eval && num_mc_runs > 1
        disp('Parallel Computing Toolbox no disponible; ejecutando Monte Carlo en serie.');
    end

for mc_iter = 1:num_mc_runs
    fprintf('\n--- Iteración Monte Carlo %d / %d ---\n', mc_iter, num_mc_runs);
    
    % 1. Aleatorizar Condiciones (Tasas de eyección realistas: entre -5 y +5 deg/s)
    if settings.initial_state == 2
        initial.omega.omega0_x = 0;
        initial.omega.omega0_y = 0;
        initial.omega.omega0_z = 0;
    else
        initial.omega.omega0_x = deg2rad(10 * rand() - 5);
        initial.omega.omega0_y = deg2rad(10 * rand() - 5);
        initial.omega.omega0_z = deg2rad(10 * rand() - 5);
    end
    
    % 2. Aleatorizar Tensor de Inercia (+/- 10% de variación gaussiana)
    err_Is = 0.10 * randn(3,3);
    err_Is = (err_Is + err_Is')/2;
    sat.Is = nominal_Is .* (1 + err_Is);
    
    % 3. Actualizar ganancia y Vector de Estado inicial
    ctrl.k_bdot = (4 * pi / orbit.period) * (1 + sin(deg2rad(orbit.inclination))) * min(diag(sat.Is)) / (B_avg^2);
    
    % Pre-calcular filtros de sensores iniciales
    B_in_ref0 = interp1(env.time, env.B_inertial, 0, 'linear', 'extrap')';
    B_filt0 = quatRotation(quatconj(initial.attitude.q0123_0'), B_in_ref0);
    
    settings.X0 = [initial.attitude.q0123_0; initial.omega.omega0_x; initial.omega.omega0_y; initial.omega.omega0_z; ...
                   0; 0; 0; ...                                 % int_err (8:10)
                   initial.omega.omega0_x; initial.omega.omega0_y; initial.omega.omega0_z; ... % w_filt (11:13)
                   B_filt0; ...                                 % B_filt (14:16)
                   0; 0; 0];                                    % bias_g (17:19)
    
    % Mostrar las condiciones iniciales en consola
    fprintf('Condiciones Iniciales establecidas:\n');
    fprintf('  Actitud inicial (Roll, Pitch, Yaw) : [%.2f, %.2f, %.2f] deg\n', initial.attitude.rpy0_deg(1), initial.attitude.rpy0_deg(2), initial.attitude.rpy0_deg(3));
    fprintf('  Tasas de giro iniciales (X, Y, Z)  : [%.2f, %.2f, %.2f] deg/s\n', rad2deg(initial.omega.omega0_x), rad2deg(initial.omega.omega0_y), rad2deg(initial.omega.omega0_z));

    % 4. Ejecutar Integrador
    if settings.stop_at_state1 && settings.initial_state ~= 2
        eventFcn = @(t,x) detumblingEvent(t, x, settings);
    else
        eventFcn = [];
    end

    if num_mc_runs == 1
        settings.hWaitbar = waitbar(0, 'Progress: 0%','Name', 'LINKU-SAT A Simulation Progress');
        opts = odeset('RelTol', 1e-6, ...
                      'OutputFcn', @(t,y,flag) odeWaitbar(t,y,flag,settings), ...
                      'Events', eventFcn);
    else
        opts = odeset('RelTol', 1e-6, 'Events', eventFcn); % Sin waitbar para acelerar calculo masivo
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

    if settings.fast_state1_eval
        omega_norm = vecnorm(x(:,5:7), 2, 2);
        idx_first_cross = find(omega_norm <= settings.detumble_threshold, 1, 'first');
        idx_stable = findStableDetumbleIndex(tout, omega_norm, settings);
        if isempty(idx_first_cross)
            t_first_cross = NaN;
        else
            t_first_cross = tout(idx_first_cross);
        end
        if isempty(idx_stable)
            t_event_first = NaN;
            fprintf('Estado 1 no verificado: no se mantuvo bajo %.2f deg/s durante %.0f s.\n', ...
                    rad2deg(settings.detumble_threshold), settings.state1_hold_time);
        else
            t_event_first = tout(idx_stable);
            fprintf('Estado 1 verificado: %.2f horas bajo %.2f deg/s durante %.0f s.\n', ...
                    t_event_first/3600, rad2deg(settings.detumble_threshold), settings.state1_hold_time);
        end

        mc_detumble_times(mc_iter) = t_event_first / 3600;
        mc_first_cross_times(mc_iter) = t_first_cross / 3600;
        mc_energy(mc_iter) = NaN;
        continue;
    end

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
quat_error = zeros(4, len_out);
quat_error_angle = zeros(1, len_out);
T_dist_rec     = zeros(3, len_out);
K_gain_rec     = zeros(1, len_out);

% Logic variables
state_ant_post = settings.initial_state;
t_event_post = -1;
t_event_first = NaN; 

for i = 1:len_out
    t_curr = tout(i);
    q_curr = x(i, 1:4)'; 
    w_curr = x(i, 5:7)'; 
    
    % --- Onboard Computer State Logic ---
    if state_ant_post == 0 && norm(w_curr) <= settings.detumble_threshold
        state_curr = 1;
        t_event_post = t_curr;
        if isnan(t_event_first), t_event_first = t_curr; end
    elseif settings.enable_alignment && state_ant_post == 1 && t_event_post >= 0 && ...
            (t_curr - t_event_post) >= settings.alignment_wait_time
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
    B_body_true_rec(:,i) = B_body_true; % Guardamos el campo real para graficar
    T_dist_rec(:,i) = disturbance(t_curr);

    % --- IMU Reconstruction (Directamente desde ode45) ---
    % Al integrarlos como variables de estado, evitamos recalcularlos numéricamente
    w_meas = x(i, 11:13)';
    B_body_meas(:,i) = x(i, 14:16)';

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
        % Quaternion feedback control for nominal alignment (State 2)
        v_vel_eci = interp1(env.time, env.Vel_inertial', t_curr, 'linear', 'extrap')';
        v_vel_body = quatRotation(quatconj(q_curr'), v_vel_eci/norm(v_vel_eci));
        pointing_error(i) = rad2deg(acos(max(min(dot([0;0;1], v_vel_body), 1), -1)));
        
        r_ref_eci = interp1(env.time, env.Pos_inertial', t_curr, 'linear', 'extrap')';
        h_ref_eci = cross(r_ref_eci, v_vel_eci);
        dq = pointingErrorQuaternion([0;0;1], v_vel_body);
        quat_error(:,i) = dq;
        quat_error_angle(i) = rad2deg(2 * atan2(norm(dq(2:4)), abs(dq(1))));
        Wr = quatRotation(quatconj(q_curr'), h_ref_eci / max(norm(r_ref_eci)^2, eps));
        Wr_dot = [0;0;0];
        T_desired = ControlFeedback_rw(sat.Is, w_meas, dq, Wr, Wr_dot, ctrl.pointing.P, ctrl.pointing.K);
        
        % Reconstruimos el Cross-Product Steering
        B_meas = B_body_meas(:,i);
        B_norm_sq = norm(B_meas)^2;
        if B_norm_sq > 1e-15
            M_cmd = cross(B_meas, T_desired) / B_norm_sq;
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

end

if settings.fast_state1_eval
    if num_mc_runs == 1
        t_hours = tout / 3600;
        omega_mag_deg = rad2deg(vecnorm(x(:,5:7), 2, 2));

        figure('Name','Detumbling: State 1 Verification','Color','w');
        hold on; grid on;
        plot(t_hours, omega_mag_deg, 'k', 'LineWidth', 1.5);
        yline(rad2deg(settings.detumble_threshold), 'r--', 'State 1 threshold');
        yline(rad2deg(settings.detumble_exit_threshold), 'm--', 'Reset threshold');
        if ~isnan(mc_first_cross_times(end))
            xline(mc_first_cross_times(end), '--b', 'First crossing', 'LabelVerticalAlignment', 'bottom');
        end
        if ~isnan(mc_detumble_times(end))
            xline(mc_detumble_times(end), '--m', 'State 1 verified', 'LabelVerticalAlignment', 'bottom');
        end
        xlabel('Time (h)');
        ylabel('|\omega| (deg/s)');
        title(sprintf('State 1 verification: %.0f s hold time', settings.state1_hold_time));
    else
        valid_times = mc_detumble_times(~isnan(mc_detumble_times));
        valid_first_cross = mc_first_cross_times(~isnan(mc_first_cross_times));
        valid_both = ~isnan(mc_first_cross_times) & ~isnan(mc_detumble_times);
        verification_delay = mc_detumble_times(valid_both) - mc_first_cross_times(valid_both);

        figure('Name','Monte Carlo: State 1 Verification','Color','w', 'Position', [100, 100, 1300, 850]);

        subplot(3,2,[1 2]);
        hold on; grid on;
        for k = 1:num_mc_runs
            plot(mc_time_hist{k}, mc_omega_hist{k}, 'LineWidth', 0.8, 'Color', [0.45 0.45 0.45]);
        end
        yline(rad2deg(settings.detumble_threshold), 'r--', 'State 1 threshold', 'LineWidth', 1.5);
        yline(rad2deg(settings.detumble_exit_threshold), 'm--', 'Reset threshold', 'LineWidth', 1.2);
        xlabel('Time (h)');
        ylabel('|\omega| (deg/s)');
        title(sprintf('Angular velocity magnitude - %d Monte Carlo runs', num_mc_runs));

        subplot(3,2,3);
        if isempty(valid_first_cross)
            text(0.5, 0.5, 'No first crossings', 'HorizontalAlignment', 'center');
            axis off;
        else
            histogram(valid_first_cross, min(15, max(3, numel(valid_first_cross))), 'FaceColor', '#4DBEEE', 'EdgeColor', 'w');
            grid on;
            xlabel('First crossing time (h)');
            ylabel('Runs');
            title(sprintf('First crossings: %d / %d', numel(valid_first_cross), num_mc_runs));
        end

        subplot(3,2,4);
        if isempty(valid_times)
            text(0.5, 0.5, 'No verified runs', 'HorizontalAlignment', 'center');
            axis off;
        else
            histogram(valid_times, min(15, max(3, numel(valid_times))), 'FaceColor', '#0072BD', 'EdgeColor', 'w');
            grid on;
            xlabel('Verified state 1 time (h)');
            ylabel('Runs');
            title(sprintf('Verified runs: %d / %d', numel(valid_times), num_mc_runs));
        end

        subplot(3,2,5);
        if isempty(verification_delay)
            text(0.5, 0.5, 'No verification delays', 'HorizontalAlignment', 'center');
            axis off;
        else
            histogram(verification_delay, min(15, max(3, numel(verification_delay))), 'FaceColor', '#77AC30', 'EdgeColor', 'w');
            grid on;
            xlabel('Verification delay after first crossing (h)');
            ylabel('Runs');
            title('Hold-time and reset effect');
        end

        subplot(3,2,6);
        axis off;
        if isempty(valid_times)
            summary_text = sprintf('First crossings: %d / %d\nVerified: 0 / %d\nHold time: %.0f s\nThreshold: %.2f deg/s\nReset: %.2f deg/s', ...
                                   numel(valid_first_cross), num_mc_runs, num_mc_runs, settings.state1_hold_time, ...
                                   rad2deg(settings.detumble_threshold), rad2deg(settings.detumble_exit_threshold));
        else
            summary_text = sprintf(['First crossings: %d / %d\nFirst mean: %.2f h\nVerified: %d / %d\nVerified mean: %.2f h\n' ...
                                    'Verified std: %.2f h\nVerified min: %.2f h\nVerified max: %.2f h\nDelay mean: %.2f h\n' ...
                                    'Hold time: %.0f s\nThreshold: %.2f deg/s\nReset: %.2f deg/s'], ...
                                   numel(valid_first_cross), num_mc_runs, mean(valid_first_cross), ...
                                   numel(valid_times), num_mc_runs, mean(valid_times), std(valid_times), ...
                                   min(valid_times), max(valid_times), mean(verification_delay), settings.state1_hold_time, ...
                                   rad2deg(settings.detumble_threshold), rad2deg(settings.detumble_exit_threshold));
        end
        text(0.05, 0.95, summary_text, 'VerticalAlignment', 'top', 'FontName', 'Consolas', 'FontSize', 11);
    end
    return;
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

if settings.save_results
    if ~isfolder(settings.results_dir)
        mkdir(settings.results_dir);
    end

    run_timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    result_file = fullfile(settings.results_dir, ['linku_run_' run_timestamp '.mat']);

    save(result_file, ...
        'tout', 'x', 't_hours', 'q0123out', 'pqrout', ...
        'xyzout', 'xyzout_dot', ...
        'B_body_meas', 'B_body_true_rec', ...
        'Torque_ctrl', 'Mag_moment', 'Currents', 'Power_inst', ...
        'state_rec', 'pointing_error', 'quat_error', 'quat_error_angle', 'T_dist_rec', 'K_gain_rec', ...
        'mc_detumble_times', 'mc_first_cross_times', 'mc_energy', ...
        'settings', 'ctrl', 'sat', 'orbit', 'initial', 'env', ...
        '-v7.3');

    fprintf('Resultados guardados en: %s\n', result_file);
end

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
    if settings.enable_alignment
        xline((t_event_first + settings.alignment_wait_time)/3600, '--c', 'Alignment', 'LabelVerticalAlignment', 'top','LineWidth',5);
    end
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
    if settings.enable_alignment
        xline((t_event_first + settings.alignment_wait_time)/3600, '--c', 'Alignment Start', 'LabelVerticalAlignment', 'bottom','LineWidth',5);
    end
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

% Figure 3: Quaternion Error
figure('Name','Quaternion Pointing Error','Color','w');
axq1 = subplot(2,1,1);
plot(t_hours, quat_error_angle, 'k', 'LineWidth', 1.5); grid on; hold on;
if ~isnan(t_event_first) && settings.enable_alignment
    xline((t_event_first + settings.alignment_wait_time)/3600, '--c', 'Alignment Start', 'LabelVerticalAlignment', 'bottom');
end
ylabel('Angle (deg)');
title('Equivalent Quaternion Error Angle');

axq2 = subplot(2,1,2);
plot(t_hours, quat_error(2:4,:)', 'LineWidth', 1.2); grid on; hold on;
if ~isnan(t_event_first) && settings.enable_alignment
    xline((t_event_first + settings.alignment_wait_time)/3600, '--c', 'Alignment Start', 'LabelVerticalAlignment', 'bottom');
end
xlabel('Time (h)');
ylabel('dq vector');
title('Quaternion Error Vector Components');
legend('dq_1','dq_2','dq_3', 'Location', 'eastoutside');
linkaxes([axq1, axq2], 'x');

% Figure 4: Environment Disturbance
figure('Name','Environmental Disturbances','Color','w');
plot(t_hours, T_dist_rec'*1e3, 'LineWidth', 1.1); grid on;
ylabel('Torque (mNm)'); xlabel('Time (h)');
title('External Disturbances (Gravity Gradient, etc.)');
legend('T_x','T_y','T_z');

% Figure 5: Torque de Control Reconstruido
figure('Name','Control Torque Analysis','Color','w');
subplot(2,1,1)
    % Plot del torque por ejes (X, Y, Z) en mNm para mejor escala
    plot(t_hours, Torque_ctrl(1,:)*1e3, 'b', 'LineWidth', 1.2); hold on; grid on;
    plot(t_hours, Torque_ctrl(2,:)*1e3, 'r', 'LineWidth', 1.2);
    plot(t_hours, Torque_ctrl(3,:)*1e3, 'g', 'LineWidth', 1.2);
    
    % Marcadores de transición de estado
    if ~isnan(t_event_first)
        xline(t_event_first/3600, '--m', 'Settling', 'LabelVerticalAlignment', 'bottom');
        if settings.enable_alignment
            xline((t_event_first + settings.alignment_wait_time)/3600, '--c', 'Alignment', 'LabelVerticalAlignment', 'bottom');
        end
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
        if settings.enable_alignment
            xline((t_event_first + settings.alignment_wait_time)/3600, '--c');
        end
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

function [detumble_time_h, first_cross_time_h, energy_j, time_h, omega_mag_deg] = runFastState1McIteration(mc_iter, initial, nominal_Is, sat, orbit, B_avg, env, disturbance, sensors, settings, ctrl, actuators)
% RUNFASTSTATE1MCITERATION Ejecuta una corrida independiente para parfor.
    rng(mc_iter, 'twister');

    initial_run = initial;
    sat_run = sat;
    ctrl_run = ctrl;
    settings_run = settings;

    if settings_run.initial_state == 2
        initial_run.omega.omega0_x = 0;
        initial_run.omega.omega0_y = 0;
        initial_run.omega.omega0_z = 0;
    else
        initial_run.omega.omega0_x = deg2rad(10 * rand() - 5);
        initial_run.omega.omega0_y = deg2rad(10 * rand() - 5);
        initial_run.omega.omega0_z = deg2rad(10 * rand() - 5);
    end

    err_Is = 0.10 * randn(3,3);
    err_Is = (err_Is + err_Is')/2;
    sat_run.Is = nominal_Is .* (1 + err_Is);

    ctrl_run.k_bdot = (4 * pi / orbit.period) * (1 + sin(deg2rad(orbit.inclination))) * min(diag(sat_run.Is)) / (B_avg^2);

    B_in_ref0 = interp1(env.time, env.B_inertial, 0, 'linear', 'extrap')';
    B_filt0 = quatRotation(quatconj(initial_run.attitude.q0123_0'), B_in_ref0);

    settings_run.X0 = [initial_run.attitude.q0123_0; ...
                       initial_run.omega.omega0_x; initial_run.omega.omega0_y; initial_run.omega.omega0_z; ...
                       0; 0; 0; ...
                       initial_run.omega.omega0_x; initial_run.omega.omega0_y; initial_run.omega.omega0_z; ...
                       B_filt0; ...
                       0; 0; 0];

    if settings_run.stop_at_state1 && settings_run.initial_state ~= 2
        eventFcn = @(t,x) detumblingEvent(t, x, settings_run);
    else
        eventFcn = [];
    end
    opts = odeset('RelTol', 1e-6, 'Events', eventFcn);

    [tout, x] = ode45(@(t,x) satellite_detumbling(t, x, sat_run, disturbance, sensors, settings_run, env, ctrl_run, actuators), ...
                      [0 settings_run.t_final], settings_run.X0, opts);

    omega_norm = vecnorm(x(:,5:7), 2, 2);
    idx_first_cross = find(omega_norm <= settings_run.detumble_threshold, 1, 'first');
    idx_stable = findStableDetumbleIndex(tout, omega_norm, settings_run);

    if isempty(idx_first_cross)
        first_cross_time_h = NaN;
    else
        first_cross_time_h = tout(idx_first_cross) / 3600;
    end

    if isempty(idx_stable)
        detumble_time_h = NaN;
    else
        detumble_time_h = tout(idx_stable) / 3600;
    end

    energy_j = NaN;
    time_h = tout / 3600;
    omega_mag_deg = rad2deg(omega_norm);
end

function x_dot = satellite_detumbling(t, x, sat, dist, sensors, settings, env, ctrl, actuators)
% SATELLITE_DETUMBLING  Dinámica rígida + control magnético PID
    
    % --- Variables Persistentes para Lógica de Estados ---
    persistent state_ant
    
    if isempty(state_ant)
        state_ant = settings.initial_state;
    end
    
    w_filt = x(11:13);        % Filtro del Giroscopio 
    B_filt = x(14:16);        % Filtro del Magnetómetro
    bias_g = x(17:19);        % Error Bias continuo
    d_int = [0;0;0];          % Derivada inicial del integrador del PID
    
    % 1) Interpolación del entorno
    B_inertial_ref = interp1(env.time, env.B_inertial, t, 'linear', 'extrap')'; 
    
    % 2) Rotación inercial -> body
    q = x(1:4);
    B_body_true = quatRotation(quatconj(q'), B_inertial_ref); 
    
    % 3) Computadora de abordo (Determina el Estado)
    if t <= eps
        state_ant = settings.initial_state;
    end
    state = onboardComputer(t, state_ant, x, settings);
    
    % Sensor (Filtros continuos integrados matemáticamente por ode45)
    [B_body_meas, d_B_filt] = imu_mag_model_continuous(B_body_true, B_filt, sensors, t);
    [w_meas, d_w_filt, d_bias_g] = imu_gyro_model_continuous(x(5:7), w_filt, bias_g, sensors, t);

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
            
            r_ref_eci = interp1(env.time, env.Pos_inertial', t, 'linear', 'extrap')';
            v_ref_eci = interp1(env.time, env.Vel_inertial', t, 'linear', 'extrap')';
            h_ref_eci = cross(r_ref_eci, v_ref_eci);
            dq = pointingErrorQuaternion(z_body, v_target_body);
            Wr = quatRotation(q_inv, h_ref_eci / max(norm(r_ref_eci)^2, eps));
            Wr_dot = [0; 0; 0];
            d_int = [0; 0; 0];
            T_desired = ControlFeedback_rw(sat.Is, w_meas, dq, Wr, Wr_dot, ctrl.pointing.P, ctrl.pointing.K);
            
            % G. CROSS-PRODUCT STEERING: Traducir Torque Deseado a Dipolo Real (Magnetorquers)
            B_norm_sq = norm(B_body_meas)^2;
            if B_norm_sq > 1e-15
                M_cmd = cross(B_body_meas, T_desired) / B_norm_sq;
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
    x_dot = [x_dot_dyn; d_int; d_w_filt; d_B_filt; d_bias_g]; % Sumamos todas las derivadas
    
    % 9) Update state
    state_ant = state;
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

function U = ControlFeedback_rw(I, W, dq, Wr, Wr_dot, P, K)
% CONTROLFEEDBACK_RW  Torque ideal de pointing por feedback de cuaternion.
    W = W(:);
    Wr = Wr(:);
    Wr_dot = Wr_dot(:);
    dq13 = dq(2:4);
    dW = W - Wr;

    U = -P * dW - K * dq13 + skewMatrix(W) * I * W + I * (Wr_dot - skewMatrix(W) * Wr);
end

function dq = pointingErrorQuaternion(body_axis, target_body)
% POINTINGERRORQUATERNION  Error que lleva body_axis hacia target_body.
    body_axis = body_axis(:) / norm(body_axis);
    target_body = target_body(:) / norm(target_body);

    err_axis = cross(body_axis, target_body);
    err_norm = norm(err_axis);
    dot_val = max(min(dot(body_axis, target_body), 1), -1);

    if err_norm < 1e-10
        if dot_val > 0
            dq = [1; 0; 0; 0];
        else
            fallback_axis = [1; 0; 0];
            if abs(dot(body_axis, fallback_axis)) > 0.9
                fallback_axis = [0; 1; 0];
            end
            err_axis = cross(body_axis, fallback_axis);
            err_axis = err_axis / norm(err_axis);
            dq = [0; -err_axis];
        end
        return;
    end

    err_axis = err_axis / err_norm;
    err_angle = atan2(err_norm, dot_val);

    % Signo elegido para que -K*dq(2:4) tenga el mismo sentido que cross(axis,target).
    dq = [cos(0.5 * err_angle); -err_axis * sin(0.5 * err_angle)];
end

function S = skewMatrix(v)
    v = v(:);
    S = [0, -v(3), v(2);
         v(3), 0, -v(1);
        -v(2), v(1), 0];
end

function [value, isterminal, direction] = detumblingEvent(t, x, settings)
% DETUMBLINGEVENT Detiene la simulacion solo si el estado 1 permanece estable.
    persistent t_state1_candidate

    if isempty(t_state1_candidate) || t <= eps
        t_state1_candidate = NaN;
    end

    omega_norm = norm(x(5:7));
    if omega_norm <= settings.detumble_threshold && isnan(t_state1_candidate)
        t_state1_candidate = t;
    elseif omega_norm > settings.detumble_exit_threshold
        t_state1_candidate = NaN;
    end

    if isnan(t_state1_candidate)
        value = 1;
    else
        value = (t - t_state1_candidate) - settings.state1_hold_time;
    end
    isterminal = settings.stop_at_state1;
    direction = 1;
end

function idx_stable = findStableDetumbleIndex(t, omega_norm, settings)
% FINDSTABLEDETUMBLEINDEX Verifica permanencia bajo umbral con histeresis.
    idx_stable = [];
    t_candidate = NaN;

    for idx = 1:numel(t)
        if omega_norm(idx) <= settings.detumble_threshold && isnan(t_candidate)
            t_candidate = t(idx);
        elseif omega_norm(idx) > settings.detumble_exit_threshold
            t_candidate = NaN;
        end

        if ~isnan(t_candidate) && (t(idx) - t_candidate) >= settings.state1_hold_time
            idx_stable = idx;
            return;
        end
    end
end

function [B_meas, B_filt_dot] = imu_mag_model_continuous(B_true, B_filt, sensors, t)
% IMU_MAG_MODEL_CONTINUOUS Simula ruido y filtro pasa-bajo en tiempo continuo
    % Usamos un ruido pseudo-aleatorio suave para evitar discontinuidades 
    % que harían que ode45 (de paso variable) reduzca su velocidad a casi cero.
    noise_nT = sensors.mag.desvEst * [sin(t*50); cos(t*60); sin(t*70)];
    B_raw = B_true(:) + noise_nT;
    
    % Filtro Pasa-Bajo Analógico en Tiempo Continuo (dy/dt = (x - y)/tau)
    B_filt_dot = (B_raw - B_filt) / sensors.mag.tau;
    B_meas = B_filt; 
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

function state = onboardComputer(t, state_ant, x, settings)
    w = x(5:7);
    threshold = deg2rad(1); % Umbral de 1 deg/s
    
    % Variables que sobreviven entre llamadas de ode45
    persistent t_event
    
    % InicializaciÃ³n de t_event si estÃ¡ vacÃ­o
    if isempty(t_event) || t <= eps
        if settings.initial_state == 1
            t_event = t;
        else
            t_event = -1;
        end
    end

    % LÓGICA DE TRANSICIÓN
    if state_ant == 0 && norm(w) <= threshold
        % HIT: Se alcanzó la velocidad por primera vez
        state = 1;
        t_event = t; % Guardamos el tiempo exacto del evento
        fprintf('At t=%.2f s: Target reached. State 0 -> 1. Timer started.\n', t);
        
    elseif state_ant == 1
        % Espera posterior al detumbling antes de pasar al modo nominal.
        if settings.enable_alignment && t_event >= 0 && (t - t_event) >= settings.alignment_wait_time
            state = 2;
            fprintf('At t=%.2f s: Settling complete. State 1 -> 2. Alignment started.\n', t);
        else
            state = 1;
        end
        
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

function [w_meas, w_filt_dot, bias_dot] = imu_gyro_model_continuous(w_true, w_filt, bias, sensors, t)
% IMU_GYRO_MODEL_CONTINUOUS Simula errores de giroscopio en tiempo continuo
    s = sensors.gyro.scaleFactor;
    m = sensors.gyro.misalign;
    
    T_sf = diag([1+s(1), 1+s(2), 1+s(3)]);
    T_ma = [1,      m(1,2), m(1,3);
            m(2,1), 1,      m(2,3);
            m(3,1), m(3,2), 1];
            
    % Ruido suave de alta frecuencia
    eta_g = sensors.gyro.noiseStd * [sin(t*55); cos(t*65); sin(t*75)];
    
    % Inestabilidad (Random walk) modelada como derivada
    bias_dot = sensors.gyro.biasWalkStd * [cos(t*10); sin(t*12); cos(t*15)];
    
    w_raw = T_ma * T_sf * w_true + bias + eta_g;
    w_sat = max(min(w_raw, sensors.gyro.yMax), sensors.gyro.yMin);
    
    % Filtro Pasa-Bajo Continuo
    w_filt_dot = (w_sat - w_filt) / sensors.gyro.tau;
    w_meas = w_filt;
end
