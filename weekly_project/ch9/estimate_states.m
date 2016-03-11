% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
  
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    persistent pn_d1 pe_d1 h_d1 static_pres_d1...
                x_accel_d1 y_accel_d1 z_accel_d1...
                course_d1 Vg_d1 diff_pres_d1...
                x_gyro_d1 y_gyro_d1 z_gyro_d1...
                P_a P_p...
                xhat_a xhat_p...
                count;
    
    if(t == 0)
        pn_d1 = 0;
        pe_d1 = 0;
        h_d1 = 100;
        static_pres_d1 = 1244;
        diff_pres_d1 = 800;
        x_accel_d1 = 0;
        y_accel_d1 = 0;
        z_accel_d1 = 0;
        course_d1 = 0;
        Vg_d1 = 35;
        x_gyro_d1 = 0;
        y_gyro_d1 = 0;
        z_gyro_d1 = 0;
        
        P_a  = [0, 0; 0, 0];
        P_p = 0;
        xhat_a = [P.phi0, P.theta0]';
        xhat_p = [P.pn0, P.pe0, P.Vg0, P.chi0, P.wind_n, P.wind_e, P.psi0]';
        count = 100;
    end
    
    %% North Location
    pn_a = .05;
    pnhat = LPF(y_gps_n, pn_d1, P.Ts_gps, pn_a);
    pn_d1 = pnhat;
    
    %% East Location
    pe_a = .05;
    pehat = LPF(y_gps_e, pe_d1, P.Ts_gps, pe_a);
    pe_d1 = pehat;
    
    %% Elevation (GPS) -- Not Used Yet
%     h_a = .05;
%     hhat = LPF(y_gps_h, h_d1, P.Ts_gps, h_a);
%     h_d1 = hhat;
%     
    
    %% Elevation (Static Pressure)
    static_pres_a = 1.975;
    static_preshat = LPF(y_static_pres, static_pres_d1, P.Ts, static_pres_a);
    static_pres_d1 = static_preshat;
    
    hhat = static_preshat/(P.gravity*P.rho);
    
    %% Differential Pressure (results in Va)
    diff_pres_a = 10;
    diff_preshat = LPF(y_diff_pres, diff_pres_d1, P.Ts, diff_pres_a);
    diff_pres_d1 = diff_preshat;
    
    Vahat = sqrt((2/P.rho)*diff_preshat);
    
    %% Y-Acc (See below for phi)
    y_accel_a = 50;
    y_accelhat = LPF(y_accel_y, y_accel_d1, P.Ts, y_accel_a);
    y_accel_d1 = y_accelhat;
    
    %% Z-Acc (for phi)
    z_accel_a = 50;
    z_accelhat = LPF(y_accel_z, z_accel_d1, P.Ts, z_accel_a);
    z_accel_d1 = z_accelhat;
    
    phihat = atan(y_accelhat/z_accelhat);
        
    %% X-Acc (for theta)
    x_accel_a = 50;
    x_accelhat = LPF(y_accel_x, x_accel_d1, P.Ts, x_accel_a);
    x_accel_d1 = x_accelhat;
    
    thetahat = asin(x_accelhat/P.gravity);
    
    %% Course
    course_a = .5;
    coursehat = LPF(y_gps_course, course_d1, P.Ts_gps, course_a);
    course_d1 = coursehat;
    chihat = coursehat;
    
    %% Ground Speed
    Vg_a = .5;
    Vghat = LPF(y_gps_Vg, Vg_d1, P.Ts_gps, Vg_a);
    Vg_d1 = Vghat;
    
    %% X-Gyro (p)
    x_gyro_a = 50;
    x_gyrohat = LPF(y_gyro_x, x_gyro_d1, P.Ts, x_gyro_a);
    x_gyro_d1 = x_gyrohat;
    phat = x_gyrohat;
    
    %% Y-Gyro (q)
    y_gyro_a = 50;
    y_gyrohat = LPF(y_gyro_y, y_gyro_d1, P.Ts, y_gyro_a);
    y_gyro_d1 = y_gyrohat;
    qhat = y_gyrohat;
    
    %% Z-Gyro (r)
    z_gyro_a = 50;
    z_gyrohat = LPF(y_gyro_z, z_gyro_d1, P.Ts, z_gyro_a);
    z_gyro_d1 = z_gyrohat;
    rhat = z_gyrohat;
    
    wnhat = 0;
    wehat = 0;
    psihat = chihat;
    
    %% Extended Kalman Filter
      
    Tout = .01;
    
    N = 10;
    
    Q_a = [(1/100000000), 0; 0, (1/1000000000)];
    
    R_a = [P.sigma_accel;...
           P.sigma_accel;...
           P.sigma_accel];
        
    for i = 1:N
       f_a = [phat+qhat*sin(xhat_a(1))*tan(xhat_a(2))+rhat*cos(xhat_a(1))*tan(xhat_a(2));...
                qhat*cos(xhat_a(1))-rhat*sin(xhat_a(1))];
            
       xhat_a = xhat_a + (Tout/N)*f_a;
       
       A_a = [qhat*cos(xhat_a(1))*tan(xhat_a(2))+rhat*sin(xhat_a(1))*tan(xhat_a(2)), (qhat*sin(xhat_a(1))-rhat*cos(xhat_a(1)))/((cos(xhat_a(2)))^2);...
            -qhat*sin(xhat_a(1))-rhat*cos(xhat_a(1)), 0];
        
       P_a = P_a + (Tout/N)*(A_a*P_a + P_a*A_a' + Q_a);
    end
    
    %% Non-GPS Sensor Update
   
    h_a = [qhat*Vahat*sin(xhat_a(2)) + P.gravity*sin(xhat_a(2));...
        rhat*Vahat*cos(xhat_a(2)) - phat*Vahat*sin(xhat_a(2)) - P.gravity*cos(xhat_a(2))*sin(xhat_a(1));...
        -qhat*Vahat*cos(xhat_a(2)) - P.gravity*cos(xhat_a(2))*cos(xhat_a(1))];
    
    C_a = [0, qhat*Vahat*cos(xhat_a(2))+P.gravity*cos(xhat_a(2));...
        -P.gravity*cos(xhat_a(1))*cos(xhat_a(2)), -rhat*Vahat*sin(xhat_a(2))-phat*Vahat*cos(xhat_a(2))+P.gravity*sin(xhat_a(1))*sin(xhat_a(2));...
        P.gravity*sin(xhat_a(1))*cos(xhat_a(2)), (qhat*Vahat+P.gravity*cos(xhat_a(1)))*sin(xhat_a(2))];
    
    h_x = h_a(1);
    C_x = C_a(1,:);
    L_x = P_a*C_x'/(R_a(1) + C_x*P_a*C_x');
    P_a = (eye(2) - L_x*C_x)*P_a;
    xhat_a = xhat_a + L_x*(y_accel_x - h_x);
    
    h_y = h_a(2);
    C_y = C_a(2,:);
    L_y = P_a*C_y'/(R_a(2) + C_y*P_a*C_y');
    P_a = (eye(2) - L_y*C_y)*P_a;
    xhat_a = xhat_a + L_y*(y_accel_y - h_y);
    
    h_z = h_a(3);
    C_z = C_a(3,:);
    L_z = P_a*C_z'/(R_a(3) + C_z*P_a*C_z');
    P_a = (eye(2) - L_z*C_z)*P_a;
    xhat_a = xhat_a + L_z*(y_accel_z - h_z);
         
    phihat   = xhat_a(1);
    thetahat = xhat_a(2);
  
    
    %% implement continous-discrete EKF to estimate pn, pe, chi, Vg
    
    Q_p = [.0000000001, 0, 0, 0, 0, 0, 0;...
            0, .0000001, 0, 0, 0, 0, 0;...
            0, 0, .00000001, 0, 0, 0, 0;...
            0, 0, 0, .0000001, 0, 0, 0;...
            0, 0, 0, 0, .0000000001, 0, 0;...
            0, 0, 0, 0, 0, .0000000001, 0;...
            0, 0, 0, 0, 0, 0, .0000000001];

    R_p = [...
        P.sigma_gps_n,...       % y_gps_n
        P.sigma_gps_e,...       % y_gps_e
        P.sigma_V,...           % y_gps_Vg
        P.sigma_course,...      % y_gps_course
        0.001,...               % pseudo measurement #1
        0.001,...               % pseudo measurement #2
        ];
    
    N = 10;
    % prediction step
    for i=1:N,
        pndot = (xhat_p(3)*cos(xhat_p(4)));
        pedot = (xhat_p(3)*sin(xhat_p(4)));
        psidot = (qhat*sin(phihat)/cos(thetahat) + rhat*cos(phihat)/cos(thetahat));
        %Vgdot = Vahat*psidot*(xhat_p(6)*cos(xhat_p(7)) - xhat_p(5)*sin(xhat_p(7)))/xhat_p(3);
        
        Vgdot = ((Vahat*cos(xhat_p(7))+xhat_p(5))*(-Vahat*psidot*sin(xhat_p(7)))+(Vahat*sin(xhat_p(7))+xhat_p(6))*(Vahat*psidot*cos(xhat_p(7))))/xhat_p(3);
        
        chidot = (P.gravity/xhat_p(3))*tan(phihat)*cos(xhat_p(4)-xhat_p(7));
        f_p = [pndot;...
                pedot;...
                Vgdot;... 
                chidot;... 
                0;...
                0;...
                psidot]; 
        xhat_p = xhat_p + (P.Ts/N)*f_p;
        
        dVgdot_dpsi = -psidot*Vahat*(xhat_p(5)*cos(xhat_p(7))+xhat_p(6)*sin(xhat_p(7)))/xhat_p(3);
        dchidot_dVg = (-P.gravity/(xhat_p(3)^2))*tan(phihat)*cos(xhat_p(4)-xhat_p(7));
        dchidot_dchi = (-P.gravity/xhat_p(3))*tan(phihat)*sin(xhat_p(4)-xhat_p(7));
        dchidot_dpsi = (P.gravity/xhat_p(3))*tan(phihat)*sin(xhat_p(4)-xhat_p(7));
        
        A_p = [0, 0, cos(xhat_p(4)), -xhat_p(3)*sin(xhat_p(4)), 0, 0, 0;...
                0, 0, sin(xhat_p(4)), xhat_p(3)*cos(xhat_p(4)), 0, 0, 0;...
                0, 0, -Vgdot/xhat_p(3), 0, -psidot*Vahat*sin(xhat_p(7)), psidot*Vahat*cos(xhat_p(7)), dVgdot_dpsi;...
                0, 0, dchidot_dVg, dchidot_dchi, 0, 0, dchidot_dpsi;...
                0, 0, 0, 0, 0, 0, 0;...
                0, 0, 0, 0, 0, 0, 0;...
                0, 0, 0, 0, 0, 0, 0];
        P_p = P_p + P.Ts_gps/N*(A_p*P_p + P_p*A_p' + Q_p);
    end
    
    %% GPS measurement updates
    
    if(count == 100),
    
        count = 0;
        
        y_wind_n = Vahat*cos(xhat_p(7)) + xhat_p(5) - xhat_p(3)*cos(xhat_p(4));
        y_wind_e = Vahat*sin(xhat_p(7)) + xhat_p(6) - xhat_p(3)*sin(xhat_p(4));
        
        h_p = [xhat_p(1);...
                xhat_p(2);...
                xhat_p(3);...
                xhat_p(4);...
                y_wind_n;...
                y_wind_e];
            
        C_p = [1, 0, 0, 0, 0, 0, 0;...
                0, 1, 0, 0, 0, 0, 0;...
                0, 0, 1, 0, 0, 0, 0;...
                0, 0, 0, 1, 0, 0, 0;...
                0, 0, -cos(xhat_p(4)), pedot, 1, 0, -Vahat*sin(xhat_p(7));...
                0, 0, -sin(xhat_p(4)), -pndot, 0, 1, Vahat*cos(xhat_p(7))];

        %North Position
        h_n = h_p(1);
        C_n = C_p(1,:);
        L_n = P_p*C_n'/(R_p(1) + C_n*P_p*C_n');
        P_p = (eye(7) - L_n*C_n)*P_p;
        %xhat_p = xhat_p + L_n*(pnhat - h_n);
        xhat_p = xhat_p + L_n*(y_gps_n - h_n);
            
        %East Position
        h_e = h_p(2);
        C_e = C_p(2,:);
        L_e = P_p*C_e'/(R_p(1) + C_e*P_p*C_e');
        P_p = (eye(7) - L_e*C_e)*P_p;
        xhat_p = xhat_p + L_e*(pehat - h_e);
        %xhat_p = xhat_p + L_e*(y_gps_e - h_e);
        
        %Ground Speed
        h_Vg = h_p(3);
        C_Vg = C_p(3,:);
        L_Vg = P_p*C_Vg'/(R_p(1) + C_Vg*P_p*C_Vg');
        P_p = (eye(7) - L_Vg*C_Vg)*P_p;
        %xhat_p = xhat_p + L_Vg*(Vghat - h_Vg);
        xhat_p = xhat_p + L_Vg*(y_gps_Vg - h_Vg);
        
        %Course (chi)
        h_chi = h_p(4);
        C_chi = C_p(4,:);
        L_chi = P_p*C_chi'/(R_p(1) + C_chi*P_p*C_chi');
        P_p = (eye(7) - L_chi*C_chi)*P_p;
        xhat_p = xhat_p + L_chi*(coursehat - h_chi);
        %xhat_p = xhat_p + L_chi*(y_gps_course - h_chi);
                
    end
     
    pnhat    = xhat_p(1);
    pehat    = xhat_p(2);
    Vghat    = xhat_p(3);
    chihat   = xhat_p(4); 
    wnhat    = xhat_p(5);
    wehat    = xhat_p(6);
    psihat   = xhat_p(7);
    
    %% Final Mapping of States
        
    bxhat = 0;
    byhat = 0;
    bzhat = 0;
  
    alphahat = 0;
    betahat = 0;
        
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
    
    count = count + 1;
end

function [filtered] = LPF(new, old, Ts, a)

    alpha = exp(-a*Ts);

    filtered = alpha*old + (1 - alpha)*new;

end
