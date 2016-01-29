% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
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
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis

    Rbv = [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);...
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta);...
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)];
    
    %wind gusts in the Body frame converted to NED frame (vehicle)
    wind_gust_ned = Rbv'*wind(4:6)
    
    % compute wind data in NED
    w_n = w_ns + wind_gust_ned(1);
    w_e = w_es + wind_gust_ned(1);
    w_d = w_ds + wind_gust_ned(1);
    
    %wind in NED converted to Body frame
    wind_body = Rbv*[w_n; w_e; w_d];
    
    % Body frame componenets of the airspeed vector
    u_r = u - wind_body(1);
    v_r = v - wind_body(2);
    w_r = w - wind_body(3);
    
    % compute air data
    Va = sqrt((u_r)^2 + (v_r)^2 + (w_r)^2);
    alpha = atan(w_r/u_r);
    beta = asin(v_r/(Va));
    
    %Functions of Alpha
    Cl = P.Clo + P.Cla*alpha;
    Cd = P.Cdo + P.Cda*alpha;
    
    Cx = -Cd*cos(alpha) + Cl*sin(alpha);
    Cxq = -P.Cdq*cos(alpha) + P.Clq*sin(alpha);
    Cxd_e = -P.Cdd_e*cos(alpha) + P.Cld_e*sin(alpha);
    
    Cz = -Cd*sin(alpha) - Cl*cos(alpha);
    Czq = -P.Cdq*sin(alpha) - P.Clq*cos(alpha);
    Czd_e = -P.Cdd_e*sin(alpha) - P.Cld_e*cos(alpha);
    
    
    % compute external forces and torques on aircraft
    Force(1) =  -P.mass*P.gravity*sin(theta)...
        +(1/2)*P.rho*(Va)^2*P.S*(Cx + Cxq*(P.c/(2*Va))*q + Cxd_e*delta_e)...
        +(1/2)*P.rho*P.Sprop*P.Cprop*((P.Kmotor*delta_t)^2 - Va^2);
    
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi)...
        +(1/2)*P.rho*(Va)^2*P.S*(P.Cyo + P.Cyb*beta + P.Cyp*(P.b/(2*Va))*p...
            + P.Cyr*(P.b/(2*Va))*r + P.Cyd_a*delta_a + P.Cyd_r*delta_r)...
        +(1/2)*P.rho*P.Sprop*P.Cprop*(0);
    
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi)...
         +(1/2)*P.rho*(Va)^2*P.S*(Cz + Czq*(P.c/(2*Va))*q + Czd_e*delta_e)...
         +(1/2)*P.rho*P.Sprop*P.Cprop*(0);
        
    
    Torque(1) = (1/2)*P.rho*(Va)^2*P.S...
        *(P.b*(P.Clo + P.Clb*beta + P.Clp*(P.b/(2*Va))*p ...
            + P.Clr*(P.b/(2*Va))*r + P.Cld_a + P.Cld_r*delta_r))...
        + (-P.Ktp*(P.Komega*delta_t)^2);
        
    Torque(2) = (1/2)*P.rho*(Va)^2*P.S...
        *(P.c*(P.Cmo + P.Cma*alpha + P.Cmq*(P.c/(2*Va))*q...
            + P.Cmd_e*delta_e))...
        + 0;
    
    Torque(3) = (1/2)*P.rho*(Va)^2*P.S...
        *(P.b*(P.Cno + P.Cnb*beta + P.Cnp*(P.b/(2*Va))*p ...
            + P.Cnr*(P.b/(2*Va))*r + P.Cnd_a + P.Cnd_r*delta_r))...
        + 0;
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



