function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)

% x_trim is the trimmed state,
% u_trim is the trimmed input
Va_trim = sqrt(x_trim(4)^2 + x_trim(5)^2 + x_trim(6)^2); % how do we define this? assume no wind? just take x-body component? or u,v,w trim to compute?
theta_trim = x_trim(8);
psi_trim = x_trim(9);
alpha_trim = atan(x_trim(6)/x_trim(4)); % how determine this???
delta_e_trim = u_trim(1);
delta_t_trim = u_trim(4);

a_phi1 = 0.5*P.rho*Va_trim^2*P.S*P.b*P.C_pp*P.b/(2*Va_trim);
a_phi2 = 0.5*P.rho*Va_trim^2*P.S*P.b*P.C_pdelta_a;
a_theta1 = -P.rho*Va_trim^2*P.c*P.S/(2*P.Jy)*P.C_mq*P.c/(2*Va_trim);
a_theta2 = -P.rho*Va_trim^2*P.c*P.S/(2*P.Jy)*P.C_malpha;
a_theta3 = P.rho*Va_trim^2*P.c*P.S/(2*P.Jy)*P.C_mdelta_e;
a_V1 = P.rho*Va_trim*P.S/P.mass*(P.C_D0 + P.C_Dalpha*alpha_trim + P.C_Ddelta_e*delta_e_trim) + P.rho*P.Sprop*P.Cprop*Va_trim/P.mass;
a_V2 = P.rho*P.Sprop/P.mass*P.Cprop*P.k_motor^2*delta_t_trim; % which k is this?
a_V3 = P.gravity*cos(theta_trim - alpha_trim); %- psi_trim);
a_beta1 = P.rho*Va_trim*P.S/(2*P.mass)*P.C_Ybeta;
a_beta2 = P.rho*Va_trim*P.S/(2*P.mass)*P.C_Ydelta_r;
    
% define transfer functions - are we not implementing sideslip dynamics? (pg 71)
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

