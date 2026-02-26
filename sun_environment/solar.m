function [sun_i,r_sun]=solar(yr,mth,day,hr,min,sec)
%
% This function computes the solar ephemeris given date and time
%
%  The inputs are:      
%         yr = year, e.g. 1995
%        mth = month, e.g. Jan=1, Feb=2, etc.
%        day = day, e.g. 1-31
%         hr = hour, e.g. 0-23
%        min = minutes, e.g. 0-59
%        sec = seconds, e.g. 0-59
%         dt = sampling interval (sec)
%         tf = run time (sec)
%  The outputs are:
%      sun_i = x,y,z components of ECI unit vector
%      r_sun = magnitude of Sun position (km) 
%

% John L. Crassidis 4/24/95 modified by bespi123

au = 149597870.0; %%Astronomical units (km)

%%%Compute Julian date and T_UT1
jd = 367*yr-fix(7*(yr+fix((mth+9)/12))/4)+fix(275*mth/9)+day+1721013.5+((sec/60+min)/60+hr)/24;
jd_cent=(jd-2451545)/36525;

% Mean longitude (deg)		
l = 280.460+36000.771*jd_cent;	
% Reduce  mean longitude for the fisrt quart
while l < 0
  l = l + 360;
end

% Mean anomaly (deg)
g = 357.5277233+35999.05034*jd_cent;		
% Reduce mean to the first quadrant
while g < 0
  g = g + 360;
end

% Ecliptic longitude (deg)
long = l + 1.914666471*sin(deg2rad(g)) + 0.019994643*sin(2*deg2rad(g));	

% Obliquity of ecliptic (deg)
e = 23.439291 - 0.0130042*jd_cent;			

% X,Y,Z components of ECI vector
sx = cos(deg2rad(long));				
sy = cos(deg2rad(e)).*sin(deg2rad(long));			
sz = sin(deg2rad(e)).*sin(deg2rad(long));

% Output
sun_i=[sx sy sz];
r_sun=(1.000140612 - 0.016708617*cos(deg2rad(g))-0.000139589*cos(2*deg2rad(g)))*au;
end