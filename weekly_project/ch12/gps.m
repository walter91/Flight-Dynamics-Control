% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
    persistent nu;
    
    % simulate random values
    sigma_Vg = P.sigma_V;
    sigma_chi = P.sigma_V/Va; %should be Vg but assume no wind?
    variance = [P.sigma_gps_n; P.sigma_gps_e; P.sigma_gps_d;...
                sigma_Vg; sigma_chi];
    eta = randn(5,1).*variance;
    
    eta_V = eta(4);
    eta_chi = eta(5);
    
    % initialize persistent variable
    if t == 0,
        nu = [0; 0; 0];
    end
    
    % construct North, East, and altitude GPS measurements
    nu = exp(-P.Ts_gps/P.K_gps_inv)*nu + eta(1:3);
    y_gps_n = pn + nu(1);
    y_gps_e = pe + nu(2); 
    y_gps_h = -pd + nu(3); 
    
    % construct groundspeed and course measurements
    y_gps_Vg     = sqrt((Va*cos(psi) + wn)^2 + (Va*sin(psi) + we)^2) + eta_V;
    y_gps_course = atan2(Va*sin(psi) + we, Va*cos(psi) + wn) + eta_chi;

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



