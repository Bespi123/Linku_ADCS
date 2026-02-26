function Tdis = simplifiedDisturbances(t)
    f = 1.28E-4;
    a = 9.33E-8;
    phase = 2*pi*rand(3,1);
    off = 1.53E-7;
    Tdis = a*sin(2*pi*f*t+phase)+off;
end