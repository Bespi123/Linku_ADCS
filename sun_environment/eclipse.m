function out=eclipse(r,r_sun)
%function out=eclipse(r,r_sun);
%
% This function determines eclipse conditions.
%
%  The inputs are:
%     r = spacecraft position (km)
%     r_sun = sun position (km)
%
%  The output is:
%       out = 1 for sun available
%           = 0 for eclipse

% John L. Crassidis 11/19/12
% Algorithm derived from Vallado's third edition book

    %%%Turn vectors to row
    r    = reshape(r, 1, []);
    [a,b]= size(r_sun);
    if a==1 || b==1
        r_sun = reshape(r_sun, 1, []);
    end

    %%Define constants
    rearth = 6378.1363  ;
    [m,~]  = size(r_sun);
    out    = zeros(m,1) ;

    % Determine Square of Vector Norms and Dot Product
    r_norm2     = (r(:,1).^2+r(:,2).^2+r(:,3).^2);
    r_sun_norm2 = (r_sun(:,1).^2+r_sun(:,2).^2+r_sun(:,3).^2);
    dot_prod    =  r(:,1).*r_sun(:,1)+r(:,2).*r_sun(:,2)+r(:,3).*r_sun(:,3);

    tau_min = (r_norm2-dot_prod)./(r_norm2+r_sun_norm2-2*dot_prod);
    out     = tau_min;

    j      = find(tau_min < 0 | tau_min > 1);
    out(j) = ones(length(j),1);

    if length(j) < m
        c_tau_min2 = (1-tau_min).*r_norm2+dot_prod.*tau_min;
        k          = find(c_tau_min2 >= rearth^2);
        out(k)     = ones(length(k),1);
    end
end

