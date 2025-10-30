function dv_com = pucp_orbit_propagator(state, params)
   % Unpack state vector:
   r_com = state(1:3);
   v_com = state(4:6);
   % Position Components
   x = r_com(1);
   y = r_com(2);
   z = r_com(3);
   r_norm = norm(r_com);
   s = z / r_norm;
   % J2 factor for Gravity Correction
   factor_J2 = (3/2) * params.J2 * (params.R_earth / r_norm)^2;

   % Compute gravitational acceleration components including J2 correction:
   a_x = -params.mu * x / r_norm^3;
   a_y = -params.mu * y / r_norm^3;
   a_z = -params.mu * z / r_norm^3;

   a_J_x = params.mu * x / r_norm^3 * (factor_J2 * (5*s^2 - 1));
   a_J_y = params.mu * y / r_norm^3 * (factor_J2 * (5*s^2 - 1));
   a_J_z = params.mu * z / r_norm^3 * (factor_J2 * (5*s^2 - 3));

   a_grav = [a_x; a_y; a_z];
   a_J2 = [a_J_x; a_J_y; a_J_z];
  
   % Atmospheric drag (Standard model for Atmosphere Density)
   alt = r_norm - params.R_earth;
   rho = params.rhoOfH(alt);
   v_atm = cross(params.omega_earth, r_com);
   v_rel = v_com - v_atm;
   v_rel_mag = norm(v_rel);
  
   a_drag = -0.5 * rho * v_rel_mag^2 * (params.Cd * params.Aeff / params.m_sat) * (v_rel / v_rel_mag);
  
%%% THIRD-BODY ACCELERATIONS
%% 
   % Sun
   omega_sun = 2*pi/(365.25*86400);  % Sun's angular rate [rad/s]
   theta_sun = omega_sun * t + params.phi_sun;
   % Assume Sun lies in the equatorial (xy) plane:
   r_sun = params.R_sun * [cos(theta_sun); sin(theta_sun); 0];
   % Compute Sun's third-body acceleration:
   a_sun = params.mu_sun * ( (r_sun - r_com) / norm(r_sun - r_com)^3 - r_sun/norm(r_sun)^3 );
  
   % Moon
   omega_moon = 2*pi/(27.321582*86400);  % Moon's angular rate [rad/s]
   theta_moon = omega_moon * t + params.phi_moon;
   incl_moon = deg2rad(5.145);             % Moon's orbit inclination relative to Earth's equator
   % Compute Moon's position (introducing a simple inclination):
   r_moon = params.R_moon * [cos(theta_moon); sin(theta_moon)*cos(incl_moon); sin(theta_moon)*sin(incl_moon)];
   % Compute Moon's third-body acceleration:
   a_moon = params.mu_moon * ( (r_moon - r_com) / norm(r_moon - r_com)^3 - r_moon/norm(r_moon)^3 );
  
   % Sum third-body effects:
   a_third = a_sun + a_moon;
  
   % Assemble all the accelerations
   dv_com = a_grav + a_J2 + a_drag + a_third;
end