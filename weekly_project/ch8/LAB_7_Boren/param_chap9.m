%% GENERAL PARAMETERS
% constants needed for wind and gust simulation
P.sigma_u = 1.06;
P.sigma_v = P.sigma_u;
P.sigma_w = 0.7;
P.Va0 = 35;
P.L_u = 200;
P.L_v = P.L_u;
P.L_w = 50;
P.wind_n = 0.5;
P.wind_e = 0.5;
P.wind_d = 0.5;
P.altitude = 100;
P.Ts = 0.01;

P.gravity = 9.81; %m/s^2

% physical parameters of airframe (from table E.2)
P.mass = 25; %kg
P.Jx   = 0.8244; %kg-m^2
P.Jy   = 1.135; %kg-m^2
P.Jz   = 1.759; %kg-m^2
P.Jxz  = 0.1204; %kg-m^2
P.S = 0.55; %m^2
P.b = 2.8956; %m
P.c = 0.18994; %m
P.Sprop = 0.2027; %m^2
P.rho = 1.2682; %kg/m^3
P.k_motor = 80;
P.k_Tp = 0;
P.k_Omega = 0;
P.e = 0.9;
P.AR = P.b^2/P.S;


% aerodynamic coefficients (from table E.2)
P.C_L0 = 0.28;
P.C_D0 = 0.03;
P.C_m0 = -0.02338;
P.C_Lalpha = 3.45;
P.C_Dalpha = 0.30;
P.C_malpha = -0.38;
P.C_Lq = 0;
P.C_Dq = 0;
P.C_mq = -3.6;
P.C_Ldelta_e = -0.36;
P.C_Ddelta_e = 0;
P.C_mdelta_e = -0.5;
P.Cprop = 1.0;
P.M = 50;
P.alpha_0 = 0.4712;
P.epsilon = 0.1592;
P.C_Dp = 0.0437;
P.C_ndelta_r = -0.032;
P.C_Y0 = 0;
P.C_ell0 = 0;
P.C_n0 = 0;
P.C_Ybeta = -0.98;
P.C_ellbeta = -0.12;
P.C_nbeta = 0.25;
P.C_Yp = 0;
P.C_ellp = -0.26;
P.C_np = 0.022;
P.C_Yr = 0;
P.C_ellr = 0.14;
P.C_nr = -0.35;
P.C_Ydelta_a = 0;
P.C_elldelta_a = 0.08;
P.C_ndelta_a = 0.06;
P.C_Ydelta_r = -0.17;
P.C_elldelta_r = 0.105;
P.C_ndelta_r = -0.032;


% convenient constants
P.Lambda   = P.Jx*P.Jz - P.Jxz^2;
P.Lambda_1 = P.Jxz*(P.Jx - P.Jy + P.Jz)/P.Lambda;
P.Lambda_2 = P.Jz*(P.Jz - P.Jy)/P.Lambda + P.Jxz^2/P.Lambda;
P.Lambda_3 = P.Jz/P.Lambda;
P.Lambda_4 = P.Jxz/P.Lambda;
P.Lambda_5 = (P.Jz - P.Jx)/P.Jy;
P.Lambda_6 = P.Jxz/P.Jy;
P.Lambda_7 = (P.Jx - P.Jy)*P.Jx/P.Lambda + P.Jxz^2/P.Lambda;
P.Lambda_8 = P.Jx/P.Lambda;

%% CALCULATE TRIM CONDITIONS
P.Va0 = 35; % desired airspeed velocity
gamma = 0*pi/180; % desired course angle
R = 150; % desired turn radius (positive --> right hand turn)

% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -P.altitude;  % initial Down position (negative altitude)
P.u0     = P.Va0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

[x_trim,P.u_trim] = compute_trim('mavsim_trim',P.Va0,gamma,R);

% trim conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -P.altitude;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate


%% LINEAR DESIGN MODELS

P.C_p0 = P.Lambda_3*P.C_ell0 + P.Lambda_4*P.C_n0;
P.C_pbeta = P.Lambda_3*P.C_ellbeta + P.Lambda_4*P.C_nbeta;
P.C_pp = P.Lambda_3*P.C_ellp + P.Lambda_4*P.C_np;
P.C_pr = P.Lambda_3*P.C_ellr + P.Lambda_4*P.C_nr;
P.C_pdelta_a = P.Lambda_3*P.C_elldelta_a + P.Lambda_4*P.C_ndelta_a;
P.C_pdelta_r = P.Lambda_3*P.C_elldelta_r + P.Lambda_4*P.C_ndelta_r;
P.C_r0 = P.Lambda_4*P.C_ell0 + P.Lambda_8*P.C_n0;
P.C_rbeta = P.Lambda_4*P.C_ellbeta + P.Lambda_8*P.C_nbeta;
P.C_rp = P.Lambda_4*P.C_ellp + P.Lambda_8*P.C_np;
P.C_rr = P.Lambda_4*P.C_ellr + P.Lambda_8*P.C_nr;
P.C_rdelta_a = P.Lambda_4*P.C_elldelta_a + P.Lambda_8*P.C_ndelta_a;
P.C_rdelta_r = P.Lambda_4*P.C_elldelta_r + P.Lambda_8*P.C_ndelta_r;

% transfer function models
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r] ...
    = compute_tf_model(x_trim,P.u_trim,P);

% state-space models
[A_lon,B_lon,A_lat,B_lat] = compute_ss_model('mavsim_trim',x_trim,P.u_trim);

%% AUTOPILOT - LOOP GAIN CALCULATIONS

P.tau = 0.01;

% Roll
P.delta_a_MAX = 45*pi/180;
P.phi_step_MAX = 15*pi/180;
[num_phi,den_phi] = tfdata(T_phi_delta_a,'v');
a_phi2 = num_phi(3);
a_phi1 = den_phi(2);
Wn_phi = sqrt(abs(a_phi2)*P.delta_a_MAX/P.phi_step_MAX);
zeta_phi = 0.707;

P.Kp_phi = P.delta_a_MAX/P.phi_step_MAX*sign(a_phi2);
P.Kd_phi = (2*zeta_phi*Wn_phi-a_phi1)/a_phi2;

% Course
W_chi = 10;
zeta_chi = 0.8;
Wn_chi = Wn_phi/W_chi;
P.phi_c_MAX = 45*pi/180;

P.Kp_chi = 2*zeta_chi*Wn_chi*P.Va0/P.gravity;
P.Ki_chi = (Wn_chi)^2*P.Va0/P.gravity;

% Pitch
P.delta_e_MAX = 45*pi/180;
P.theta_step_MAX = 10*pi/180;
[num_theta,den_theta] = tfdata(T_theta_delta_e,'v');
a_theta1 = den_theta(2);
a_theta2 = den_theta(3);
a_theta3 = num_theta(3);
Wn_theta = sqrt(a_theta2 + P.delta_e_MAX/P.theta_step_MAX*abs(a_theta3));
zeta_theta = .707;

P.Kp_theta = P.delta_e_MAX/P.theta_step_MAX*sign(a_theta3);
P.Kd_theta = (2*zeta_theta*Wn_theta - a_theta1)/a_theta3;

% Airspeed with Throttle
P.delta_t_c_MAX = 1;
[num_V,den_V] = tfdata(T_Va_delta_t,'v');
a_V1 = den_V(2);
a_V2 = num_V(2);
Wn_V = 5;
zeta_V = 0.8;

P.Kp_V = (2*zeta_V*Wn_V - a_V1)/a_V2;
P.Ki_V = (Wn_V)^2/a_V2;

% Airspeed with Pitch
W_V2 = 17;
[num_V2,den_V2] = tfdata(T_Va_theta,'v');
a_V1 = den_V2(2);
K_thetaDC = 1;%num_Va(2)/a_V1;
Wn_V2 = Wn_theta/W_V2;
zeta_V2 = 1;
P.theta_c_MAX = 25*pi/180;

P.Kp_V2 = (a_V1 - 2*zeta_V2*Wn_V2)/(K_thetaDC*P.gravity);
P.Ki_V2 = -(Wn_V2)^2/(K_thetaDC*P.gravity);

% Altitude with Pitch
W_h = 17;
Wn_h = Wn_theta/W_h;
zeta_h = 1;%0.707;

P.Kp_h = 2*zeta_h*Wn_h/(K_thetaDC*P.Va0);
P.Ki_h = (Wn_h)^2/(K_thetaDC*P.Va0);

% Altitude State Machine
P.altitude_take_off_zone = 20;
P.altitude_hold_zone = 25;
P.theta_takeoff = 25*pi/180;

%% SENSORS
P.sigma_gyro = 0.13; %deg/s
P.sigma_accel = 0.0025*P.gravity;
P.beta_abs = 0.125; %kPa
P.beta_diff = 0.020; %kPa
P.sigma_abs = 0.01; %kPa
P.sigma_diff = 0.002; %kPa

% gps
P.sigma_gps_n = 0.21;
P.sigma_gps_e = 0.21;
P.sigma_gps_d = 0.40;
P.sigma_V = 0.04; %m/s
P.sigma_course = 3*pi/180; %rad
P.K_gps_inv = 1100;
P.Ts_gps = 1;

%% STATE ESTIMATION
P.Tout = 0.01; %P.Ts;

%% GUIDANCE MODEL
P.b_chidot = 2.1;
P.b_chi = 3.5;
P.b_hdot = 0.8;
P.b_h = 1.8;
P.b_Va = 2.2;
P.b_phi = 1;