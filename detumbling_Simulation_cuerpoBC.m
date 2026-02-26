clear; clc; close all;

%% 1) PARÁMETROS DEL SATÉLITE (LINKU - cuerpo BC)
sat.Is = diag([0.0444, 0.0444, 0.0342]);
sat.mass = 5.5; % kg

%% Actuadores: Magnetorquers (Custom Model)
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
               initial.omega.omega0_z];

%% Escenario Satelital (SGP4)
sc = satelliteScenario(settings.startTime, settings.startTime + seconds(settings.t_final), settings.sample_step);
sat.satSGP4 = satellite(sc, orbit.semiMajorAxis, orbit.eccentricity, orbit.inclination, ...
        orbit.rightAscensionOfAscendingNode, orbit.argumentOfPeriapsis, orbit.trueAnomaly);

%% 5) PRE-CÁLCULO DEL ENTORNO (TABLA)
disp('Pre-calculando entorno...');
[pos_eci, ~, time_array] = states(sat.satSGP4, "CoordinateFrame", "inertial");
env.Pos_inertial = pos_eci; % 3xN
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

%% 7) POST-PROCESO (RECONSTRUCCIÓN)
disp('Reconstruyendo variables...');

len_out = length(tout);

B_body_true    = zeros(3, len_out);
B_body_meas    = zeros(3, len_out);
Torque_ctrl    = zeros(3, len_out);
Mag_moment     = zeros(3, len_out);
Currents       = zeros(3, len_out);
Power_inst     = zeros(1, len_out);
K_gain         = zeros(1, len_out);
dis_v          = zeros(3, len_out);

for i = 1:len_out
    t_curr = tout(i);
    q_curr = x(i, 1:4)'; % 4x1
    w_curr = x(i, 5:7)'; % 3x1

    % entorno (B "inercial" aproximado)
    B_in_ref = interp1(env.time, env.B_inertial, t_curr, 'linear', 'extrap')'; % 3x1
    dip_ref  = interp1(env.time, env.dip,        t_curr, 'linear', 'extrap');

    % rotación
    B_body_true(:,i) = quatRotation(quatconj(q_curr'), B_in_ref);

    % sensor
    B_body_meas(:,i) = mag_model(B_body_true(:,i), sensors.mag.desvEst, sensors.mag.res);

    % ganancia
    min_inertia = min(diag(sat.Is));
    omega_orbit = 2*pi/settings.orbit_period;
    k_val = 2 * omega_orbit * (1 + sin(deg2rad(dip_ref))) * min_inertia * 8e9;
    K_gain(i) = k_val;

    % control
    [T_applied, M_applied] = detumblingControl([q_curr; w_curr], k_val, B_body_meas(:,i));
    Torque_ctrl(:,i) = T_applied;
    Mag_moment(:,i)  = M_applied;

    % corrientes (modelo simplificado)
    curr_vec = M_applied / (actuators.magnetorquer.n * actuators.magnetorquer.A);
    Currents(:,i) = curr_vec;

    % potencia instantánea (3 ejes a 5V)
    Power_inst(i) = sum(abs(curr_vec)) * actuators.magnetorquer.voltage;

    % perturbaciones
    dis_v(:,i) = disturbance(t_curr);
end

% compatibilidad con tus plots
Torque_mag_v   = Torque_ctrl;
m_mag_v        = Mag_moment;
mag_currents   = Currents;
mag_field_meas = B_body_meas;
k_v            = K_gain;

disp('Datos recuperados.');

%% 8) PLOTS
xyzout = interp1(env.time, env.Pos_inertial', tout, 'linear', 'extrap');
q0123out = x(:,1:4);
ptpout = quat2eul(q0123out);
pqrout = x(:,5:7);
magnOmega = vecnorm(pqrout, 2, 2);

idx_stable = round(length(tout) * 0.8);
if idx_stable < 1, idx_stable = 1; end

% Plot 1: Posición
figure('Name','Position');
h1 = plot(tout/3600, xyzout(:,1)/1E3, 'b-','LineWidth',2); hold on; grid on;
h2 = plot(tout/3600, xyzout(:,2)/1E3, 'r-','LineWidth',2);
h3 = plot(tout/3600, xyzout(:,3)/1E3, 'g-','LineWidth',2);
xlabel('Time (h)'); ylabel('Position (km)');
legend([h1,h2,h3], 'X_{ECI}', 'Y_{ECI}', 'Z_{ECI}', 'Location','eastoutside');
title('Satellite Position (Inertial Frame)');

% Plot 2: Velocidad angular
figure('Name','Angular Velocity');
[stx, ~] = obtainStableTime(tout/3600, rad2deg(pqrout(:,1)), -0.1, 0.1);
[sty, ~] = obtainStableTime(tout/3600, rad2deg(pqrout(:,2)), -0.1, 0.1);
[stz, ~] = obtainStableTime(tout/3600, rad2deg(pqrout(:,3)), -0.1, 0.1);

perc_x = percentageOfSignal(rad2deg(pqrout(idx_stable:end,1)), -0.1, 0.1);
perc_y = percentageOfSignal(rad2deg(pqrout(idx_stable:end,2)), -0.1, 0.1);
perc_z = percentageOfSignal(rad2deg(pqrout(idx_stable:end,3)), -0.1, 0.1);

p1 = subplot(2,1,1);
h1 = plot(tout/3600, rad2deg(pqrout(:,1)), 'b-','LineWidth',1.5); hold on; grid on
h2 = plot(tout/3600, rad2deg(pqrout(:,2)), 'r-','LineWidth',1.5);
h3 = plot(tout/3600, rad2deg(pqrout(:,3)), 'g-','LineWidth',1.5);
yline(0.1, 'k--'); yline(-0.1, 'k--');
xlabel('Time (h)'); ylabel('\omega (deg/s)');
legend([h1,h2,h3], '\omega_x', '\omega_y', '\omega_z', 'Location','eastoutside');
title(sprintf('Angular Rate (Estabilidad fin: X=%.1f%%, Y=%.1f%%, Z=%.1f%%)', perc_x, perc_y, perc_z));

p2 = subplot(2,1,2);
plot(tout/3600, rad2deg(magnOmega), 'k', 'LineWidth', 2); hold on; grid on;
yline(0.5, 'r--', 'Requerimiento (0.5 deg/s)');
xlabel('Time (h)'); ylabel('|\omega| (deg/s)');
title('Angular Rate Magnitude');
linkaxes([p1,p2],'x');

% Plot 3: Entorno
figure('Name','Environment');
p1 = subplot(2,1,1);
plot(tout/3600, dis_v'*1E3, '.'); grid on;
ylabel('Disturbances (mNm)'); legend('Tx','Ty','Tz');
title('Simulated Disturbances');

p2 = subplot(2,1,2);
plot(tout/3600, mag_field_meas'*1E9, '.'); grid on;
xlabel('Time (h)'); ylabel('Magnetic Field (nT)');
legend('Bx','By','Bz','Location','eastoutside');
title('Measured Magnetic Field');
linkaxes([p1,p2],'x');

% Plot 4: Actuadores
figure('Name','Actuators');
p1 = subplot(3,1,1);
plot(tout/3600, Torque_mag_v', 'LineWidth', 1.5); grid on;
ylabel('Torque (Nm)'); title('Magnetic Torque'); legend('Tx','Ty','Tz');

p2 = subplot(3,1,2);
plot(tout/3600, m_mag_v', 'LineWidth', 1.5); grid on;
ylabel('Moment (A m^2)'); title('Dipole Moment');

p3 = subplot(3,1,3);
plot(tout/3600, mag_currents', 'LineWidth', 1.5); grid on;
xlabel('Time (h)'); ylabel('Current (A)'); title('Magnetorquer Current');
linkaxes([p1,p2,p3],'x');

% Plot 5: Potencia por eje
figure('Name','Power');
volts = actuators.magnetorquer.voltage;
power_instant_xyz = abs(mag_currents)' * volts; % Nx3
plot(tout/3600, power_instant_xyz, 'LineWidth', 1.5); grid on;
legend('Px','Py','Pz','Location','eastoutside');
xlabel('Time (h)'); ylabel('Power (W)');
title(sprintf('Power Consumption (Voltage = %.1f V)', volts));

% Potencia media (usando corriente^2 integrada)
try
    avgIx2 = potenciaMedia(tout, mag_currents(1,:)');
    avgIy2 = potenciaMedia(tout, mag_currents(2,:)');
    avgIz2 = potenciaMedia(tout, mag_currents(3,:)');
    % Si quieres potencia media en W: P = V * mean(|I|) o P=R*mean(I^2) si conoces R.
    fprintf('Media de I^2 (x,y,z): [%.3e, %.3e, %.3e] A^2\n', avgIx2, avgIy2, avgIz2);
catch
    disp('Nota: potenciaMedia no encontrada o error.');
end

% Plot 6: Ganancia K
figure('Name','Control Gain');
plot(tout/3600, k_v, 'b-', 'LineWidth', 2); grid on;
xlabel('Time (h)'); ylabel('Gain K');
title('Adaptive Control Gain');

disp('Listo.');

%% ==================== FUNCIONES LOCALES ====================

function x_dot = satellite_detumbling(t, x, sat, dist, sensors, settings, env)
% SATELLITE_DETUMBLING  Dinámica rígida + control magnético tipo B-dot
% x = [q(4x1); w(3x1)]
% q: cuaternión (scalar-first) cuerpo respecto a inercial
% w: velocidad angular cuerpo [rad/s]
%
% env.time       (Nx1) segundos desde start
% env.B_inertial (Nx3) campo magnético de referencia [T] (aprox)
% env.dip        (Nx1) dip angle [deg]

    % 1) Interpolación del entorno
    B_inertial_ref = interp1(env.time, env.B_inertial, t, 'linear', 'extrap')'; % 3x1
    dip_ref        = interp1(env.time, env.dip,        t, 'linear', 'extrap');

    % 2) Rotación inercial -> body
    q = x(1:4);
    B_body_true = quatRotation(quatconj(q'), B_inertial_ref); % 3x1

    % 3) Sensor (ruido + cuantización)
    B_body_meas = mag_model(B_body_true, sensors.mag.desvEst, sensors.mag.res); % 3x1

    % 4) Ganancia K
    min_inertia = min(diag(sat.Is));
    omega_orbit = 2*pi/settings.orbit_period;
    k = 2 * omega_orbit * (1 + sin(deg2rad(dip_ref))) * min_inertia * 8e9;

    % 5) Control
    [T_control, ~] = detumblingControl(x, k, B_body_meas); % 3x1

    % 6) Disturbios
    T_dist = dist(t); T_dist = T_dist(:);

    % 7) Dinámica
    w = x(5:7);
    T_total = T_dist + T_control;

    Xi = [-q(2) -q(3) -q(4);
           q(1) -q(4)  q(3);
           q(4)  q(1) -q(2);
          -q(3)  q(2)  q(1)];
    q_dot = 0.5 * Xi * w;

    w_dot = sat.Is \ (T_total - cross(w, sat.Is*w));

    x_dot = [q_dot; w_dot];
end

function [Tc, muB_sat] = detumblingControl(state, k, B_body)
% DETUMBLINGCONTROL  Ley tipo B-dot: mu = k (w x B), Tc = mu x B
    omega = state(5:7); omega = omega(:);
    B_body = B_body(:);

    muB = k * cross(omega, B_body);

    % Saturación por dipolo máximo (0.2 A m^2)
    muB_sat = max(min(muB, 0.1), -0.1);

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