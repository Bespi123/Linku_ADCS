%%% INITIAL CONDITIONS FROM TLE %%%
a    = 6992e3;               % semi-major axis [m]
e    = 0.0037118;            % eccentricity
i    = deg2rad(97.799);      % inclination [rad]
RAAN = deg2rad(80.477);      % RAAN [rad]
Omega= deg2rad(65.443);      % argument of perigee [rad]
f    = deg2rad(294.681705);  % true anomaly [rad]

%%% PARAMETERS & CONSTANTS
params.mu       = 3.986e14;        % Earth's gravitational parameter [m^3/s^2]
params.R_earth  = 6378e3;          % Earth's radius [m]
params.J2       = 1.08263e-3;      % Earth´s J2 coefficient (approx.)
params.omega_earth = [0, 0, 7.2921E-5]'; % Velocidad angular de la tierra

% Third-body gravitational parameters
params.mu_sun  = 1.32712440018e20; % Sun's gravitational parameter (m^3/s^2)
params.mu_moon = 4.9048695e12;     % Moon's gravitational parameter (m^3/s^2)
% Average distances (m)
params.R_sun   = 1.496e11;         % Average Sun-Earth distance
params.R_moon  = 384400e3;         % Average Moon-Earth distance
% Phase offsets
params.phi_sun  = 0;               %
params.phi_moon = 0;               %

%%% Atmospheric Density Data
alts = [700,675,650,625,600,575,550,525,500,475,450,425,400,375,350,325,300,275,250,225,200,175,150,125,100];
rhos = [3.1e-14,4.1e-14,5.7e-14,7.9e-14,1.1e-13,1.6e-13,2.4e-13,3.5e-13,5.2e-13,7.7e-13,1.2e-12,1.8e-12,2.8e-12,4.4e-12,7e-12,1.1e-11,...
       1.9e-11,3.3e-11,6e-11,1.2e-10,2.5e-10,6.3e-10,2.1e-9,1.1e-8,5.6e-7];
log_rhos = log(rhos);
params.logRhoInterp = @(h_km) interp1(alts, log_rhos, h_km, 'pchip', 'extrap');
params.rhoOfH = @(h) exp(params.logRhoInterp(h/1000));





