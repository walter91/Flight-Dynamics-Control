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
    
    persistent nu_n_0;
    persistent nu_e_0;
    persistent nu_h_0;
    
    if(isempty(nu_n_0))
       nu_n_0 = 0;
       nu_e_0 = 0;
       nu_h_0 = 0;
    end
    
 
    nu_n_1 = exp(-(1/1100)*P.Ts_gps)*nu_n_0 + normrnd(0, .21);
    nu_e_1 = exp(-(1/1100)*P.Ts_gps)*nu_n_0 + normrnd(0, .21);
    nu_h_1 = exp(-(1/1100)*P.Ts_gps)*nu_n_0 + normrnd(0, .4);
    
    nu_n_0 = nu_n_1;
    nu_e_0 = nu_e_1;
    nu_h_0 = nu_h_1;
    
    % construct North, East, and altitude GPS measurements
    y_gps_n = pn + nu_n_0;
    y_gps_e = pe + nu_e_0; 
    y_gps_h = -pd + nu_h_0; 
    
    % construct groundspeed and course measurements
    y_gps_Vg     = sqrt((Va*cos(psi) + wn)^2 + (Va*sin(psi) + we)^2) + normrnd(0, .21);
    y_gps_course = atan2(Va*sin(psi) + we, Va*cos(psi) + wn) + normrnd(0, .21/y_gps_Vg);

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



