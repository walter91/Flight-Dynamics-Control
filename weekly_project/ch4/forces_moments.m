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
    pn      = x(1); %north positions, NED
    pe      = x(2); %east position, NED
    pd      = x(3); %down position, NED
    u       = x(4); %forward velocity, BODY
    v       = x(5); %right velocity, BODY
    w       = x(6); %down velocity, BODY 
    phi     = x(7); %roll, vehicle 2
    theta   = x(8); %pitch, vehicle 1
    psi     = x(9); %yaw(heading), vehicle (inertial direction) 
    p       = x(10); %roll rate, BODY
    q       = x(11); %pitch rate, BODY
    r       = x(12); %yaw rate, BODY
    delta_e = delta(1); %elevator posistion
    delta_a = delta(2); %aleron position
    delta_r = delta(3); %rudder position
    delta_t = delta(4); %throttle position
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis

    %Rotation from vehicle (inertial aligned) to body frame (eq. 2.5 on pg 15 of book)
    Rbv = [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);...
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta);...
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)];
    
    %wind gusts in the Body frame converted to NED frame (vehicle)
    wind_gust_ned = Rbv'*[u_wg; v_wg; w_wg];
    
    % compute wind data in NED by summing NED steady with NED gust from
    % above.
    w_n = w_ns + wind_gust_ned(1);
    w_e = w_es + wind_gust_ned(2);
    w_d = w_ds + wind_gust_ned(3);
    
    %wind in NED converted to Body frame
    wind_body = Rbv*[w_n; w_e; w_d];
    
    % Body frame componenets of the airspeed vector (top of pg. 57)
    u_r = u - wind_body(1);
    v_r = v - wind_body(2);
    w_r = w - wind_body(3);
    
    % compute air data (middle of pg. 57)
    Va = sqrt((u_r)^2 + (v_r)^2 + (w_r)^2);
    alpha = atan(w_r/u_r);
    beta = asin(v_r/(Va));
    
    %Linear Functions of Alpha
    %Cl = P.CLo + P.Cla*alpha; %eq 4.12, pg. 49
    %Cd = P.Cdo + P.Cda*alpha; %eq 4.13, pg. 49
    
    %Non-Linear Functions of Alpha
    sigma_alpha = (1 + exp(-P.M*(alpha - P.alpha0)) ...
        + exp(P.M*(alpha + P.alpha0))) /...
        ((1 + exp(-P.M*(alpha - P.alpha0)))*(1 + exp(P.M*(alpha + P.alpha0)))); %eq. 4.10, pg. 47
    
    Cl = (1 - sigma_alpha)*(P.CLo + P.Cla*alpha)...
        + sigma_alpha*(2*sign(alpha)*((sin(alpha))^2)*cos(alpha)); %eq. 4.9, pg. 47
    
    Cd = P.Cdp + (((P.CLo + P.Cla*alpha)^2)/(pi*P.e*(((P.b)^2)/P.S)));  %eq. 4.11, pg. 48
    
    %Other Functions of Alpha, eq. 4.19, pg. 58
    Cx = -Cd*cos(alpha) + Cl*sin(alpha);
    Cxq = -P.Cdq*cos(alpha) + P.Clq*sin(alpha);
    Cxd_e = -P.Cdd_e*cos(alpha) + P.Cld_e*sin(alpha);
    
    Cz = -Cd*sin(alpha) - Cl*cos(alpha);
    Czq = -P.Cdq*sin(alpha) - P.Clq*cos(alpha);
    Czd_e = -P.Cdd_e*sin(alpha) - P.Cld_e*cos(alpha);
    
    
    % compute external forces, eq. 4.18, pg. 57
    Force(1) =  -P.mass*P.gravity*sin(theta)...
        +(1/2)*P.rho*((Va)^2)*P.S*(Cx + Cxq*(P.c/(2*Va))*q + Cxd_e*delta_e)...
        +(1/2)*P.rho*P.Sprop*P.Cprop*((P.Kmotor*delta_t)^2 - Va^2);
    
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi)...
        +(1/2)*P.rho*((Va)^2)*P.S*(P.Cyo + P.Cyb*beta + P.Cyp*(P.b/(2*Va))*p...
            + P.Cyr*(P.b/(2*Va))*r + P.Cyd_a*delta_a + P.Cyd_r*delta_r)...
        +(1/2)*P.rho*P.Sprop*P.Cprop*(0);
    
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi)...
         +(1/2)*P.rho*((Va)^2)*P.S*(Cz + Czq*(P.c/(2*Va))*q + Czd_e*delta_e)...
         +(1/2)*P.rho*P.Sprop*P.Cprop*(0);
        
    % compute external torques, eq. 4.20, pg. 58
    Torque(1) = (1/2)*P.rho*((Va)^2)*P.S...
        *(P.b*(P.Clo + P.Clb*beta + P.Clp*(P.b/(2*Va))*p ...
            + P.Clr*(P.b/(2*Va))*r + P.Cld_a*delta_a + P.Cld_r*delta_r))...
        + (-P.Ktp*(P.Komega*delta_t)^2);
        
    Torque(2) = (1/2)*P.rho*((Va)^2)*P.S...
        *(P.c*(P.Cmo + P.Cma*alpha + P.Cmq*(P.c/(2*Va))*q...
            + P.Cmd_e*delta_e))...
        + 0;
    
    Torque(3) = (1/2)*P.rho*((Va)^2)*P.S...
        *(P.b*(P.Cno + P.Cnb*beta + P.Cnp*(P.b/(2*Va))*p ...
            + P.Cnr*(P.b/(2*Va))*r + P.Cnd_a*delta_a + P.Cnd_r*delta_r))...
        + 0;
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



