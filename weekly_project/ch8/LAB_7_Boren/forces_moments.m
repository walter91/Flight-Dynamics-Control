% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   TO-DOs:
%   - check wind data computations 
%   - check that outputs are in proper reference frame
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    % note: gust values are actually input in inertial frame. Rotation is accomplished in the 'compute wind data in NED' set
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    %% COMPUTE WIND DATA IN NED  
    % note: we sum the wind steady-state and gust values before rotating into the body frame because both arrive in the inertial frame
    
    % body-frame components of overall wind vector (steady-state and gust)(pg 56)
    u_w = (w_ns+u_wg)*cos(theta)*cos(psi) + (w_es+v_wg)*cos(theta)*sin(psi) - (w_ds+w_wg)*sin(theta);
    v_w = (w_ns+u_wg)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + (w_es+v_wg)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + (w_ds+w_wg)*sin(phi)*cos(theta);
    w_w = (w_ns+u_wg)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) + (w_es+v_wg)*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) + (w_ds+w_wg)*cos(phi)*cos(theta);
    
    % body-frame components of airspeed vector (pg 57)
    u_r = u - u_w;
    v_r = v - v_w;
    w_r = w - w_w;
    
    % Is this correct? would it not just be the sum w_ns+u_wg etc...?
    w_n = w_ns + u_wg; %u_r;
    w_e = w_es + v_wg; %v_r;
    w_d = w_ds + w_wg; %w_r;
    
    %% COMPUTE AIR DATA
    Va = sqrt(u_r^2 + v_r^2 + w_r^2); % Velocity with respect to airmass (pg 57)
    alpha = atan(w_r/u_r); % Angle of attack (pg 57)
    beta = asin(v_r/Va); % Sideslip angle (pg 57)
    
    % calculate sigmoid function to blend between linear and non-linear models (pg 47)
    sigma = (1+exp(-P.M*(alpha-P.alpha_0))+exp(P.M*(alpha+P.alpha_0)))/((1+exp(-P.M*(alpha-P.alpha_0)))*(1+exp(P.M*(alpha+P.alpha_0))));
    
    % calculate lift coefficient as function of angle of attack (pg 47)
    CL = (1-sigma)*(P.C_L0 + P.C_Lalpha*alpha) + sigma*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    
    % calculate drag coefficient (pg 48)
    CD = P.C_Dp + (P.C_L0 + P.C_Lalpha*alpha)^2/(pi*P.e*P.AR);
    
    % compute intermediate values (pg 58)
    CX = -CD*cos(alpha) + CL*sin(alpha);
    CX_q = -P.C_Dq*cos(alpha) + P.C_Lq*sin(alpha);
    CX_delta_e = -P.C_Ddelta_e*cos(alpha) + P.C_Ldelta_e*sin(alpha);
    CZ = -CD*sin(alpha) - CL*cos(alpha);
    CZ_q = -P.C_Dq*sin(alpha) - P.C_Lq*cos(alpha);
    CZ_delta_e = -P.C_Ddelta_e*sin(alpha) - P.C_Ldelta_e*cos(alpha);
    
    %% COMPUTE EXTERNAL FORCES AND TORQUES ON UAV (in body frame???)
    
    % forces (fx,fy,fz) (pg 57)
    Force(1) =  -P.mass*P.gravity*sin(theta) + 0.5*P.rho*Va^2*P.S*(CX + CX_q*P.c*q/(2*Va) + CX_delta_e*delta_e) + 0.5*P.rho*P.Sprop*P.Cprop*((P.k_motor*delta_t)^2-Va^2);
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi) + 0.5*P.rho*Va^2*P.S*(P.C_Y0 + P.C_Ybeta*beta + P.C_Yp*P.b*p/(2*Va) + P.C_Yr*P.b*r/(2*Va) + P.C_Ydelta_a*delta_a + P.C_Ydelta_r*delta_r);
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi) + 0.5*P.rho*Va^2*P.S*(CZ + CZ_q*P.c*q/(2*Va) + CZ_delta_e*delta_e);
    
    % torques (l,m,n) (pg 58)
    Torque(1) = 0.5*P.rho*Va^2*P.S*P.b*(P.C_ell0 + P.C_ellbeta*beta + P.C_ellp*P.b*p/(2*Va) + P.C_ellr*P.b*r/(2*Va) + P.C_elldelta_a*delta_a + P.C_elldelta_r*delta_r) + (-P.k_Tp*(P.k_Omega*delta_t)^2);
    Torque(2) = 0.5*P.rho*Va^2*P.S*P.c*(P.C_m0 + P.C_malpha*alpha + P.C_mq*P.c*q/(2*Va) + P.C_mdelta_e*delta_e);   
    Torque(3) = 0.5*P.rho*Va^2*P.S*P.b*(P.C_n0 + P.C_nbeta*beta + P.C_np*P.b*p/(2*Va) + P.C_nr*P.b*r/(2*Va) + P.C_ndelta_a*delta_a + P.C_ndelta_r*delta_r);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



