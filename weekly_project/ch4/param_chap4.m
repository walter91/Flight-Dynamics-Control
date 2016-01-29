P.gravity = 9.81;
   
%physical parameters of airframe
P.mass = 13.5;  %kg
P.Jx   = .8244; %kg*m^2
P.Jy   = 1.135; %kg*m^2
P.Jz   = 1.759; %kg*m^2
P.Jxz  = 1.204; %kg*m^2

P.Gamma = (P.Jx*P.Jz) - (P.Jxz)^2;
P.Gamma1 = (P.Jxz*(P.Jx - P.Jy + P.Jz))/P.Gamma;
P.Gamma2 = (P.Jz*(P.Jz - P.Jy) + (P.Jxz)^2)/P.Gamma;
P.Gamma3 = P.Jz/P.Gamma;
P.Gamma4 = P.Jxz/P.Gamma;
P.Gamma5 = (P.Jz - P.Jx)/P.Jy;
P.Gamma6 = P.Jxz/P.Jy;
P.Gamma7 = ((P.Jz - P.Jy)*P.Jx + (P.Jxz)^2)/P.Gamma;
P.Gamma8 = P.Jx/P.Gamma;

% initial conditions
P.pn0    =  0; % initial North position
P.pe0    =  0; % initial East position
P.pd0    =  0; % initial Down position (negative altitude)
P.u0     =  5; % initial velocity along body x-axis
P.v0     =  0; % initial velocity along body y-axis
P.w0     =  0; % initial velocity along body z-axis
P.phi0   =  0; % initial roll angle
P.theta0 =  0; % initial pitch angle
P.psi0   =  0; % initial yaw angle
P.p0     =  0; % initial body frame roll rate
P.q0     =  0; % initial body frame pitch rate
P.r0     =  0; % initial body frame yaw rate

% More Parameters (Appendix E)
P.S = .55;      %m^2
P.b = 2.8956;   %m
P.c = 0.18994;      %m
P.Sprop = .2027;    % m^2    
P.rho = 1.2682; %kg/m^2
P.Kmotor = 80;  %unitless
P.Ktp = 0;      %unitless
P.Komega = 0;   %unitless
P.e = .9;       %unitless

% Longitudinal Coef.
P.Clo = .28;
P.Cdo = .03;
P.Cmo = -0.02338;
P.Cla = 3.45;
P.Cda = .30;
P.Cma = -3.8;
P.Clq = 0;
P.Cdq = 0;
P.Cmq = -3.6;
P.Cld_e = -.36;
P.Cdd_e = 0;
P.Cmd_e = -.5;
P.Cprop = 1.0;
P.M = 50;
P.alpha0 = .4712;
P.epsilon = .1592;
P.Cdp = .0437;
P.Cnd_r = -.032;

% Lateral Coef.
P.Cyo = 0;
P.Clo = 0;
P.Cno = 0;
P.Cyb = -.98;
P.Clb = -.12;
P.Cnb = .25;
P.Cyp = 0;
P.Clp = -.26;
P.Cnp = .022;
P.Cyr = 0;
P.Clr = .14;
P.Cnr = -.35;
P.Cyd_a = 0;
P.Cld_a = .08;
P.Cnd_a = .06;
P.Cyd_r = -.17;
P.Cld_r = .105;

% Wind for Simulation
P.wind_n = 0;
P.wind_e = 0;
P.wind_d = 0;

%Other simulation data
P.Va0 = 5;
P.sigma_u = 1.06;
P.sigma_v = 1.06;
P.sigma_w = .7;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;

P.Ts = .1;

