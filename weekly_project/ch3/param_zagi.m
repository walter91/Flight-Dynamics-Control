P.gravity = 9.81;
   
%physical parameters of airframe
P.mass = 1.56;  %kg
P.Jx   = .1147; %kg*m^2
P.Jy   = .0576; %kg*m^2
P.Jz   = .1712; %kg*m^2
P.Jxz  = .0015; %kg*m^2

% initial conditions
P.pn0    =   % initial North position
P.pe0    =   % initial East position
P.pd0    =   % initial Down position (negative altitude)
P.u0     =   % initial velocity along body x-axis
P.v0     =   % initial velocity along body y-axis
P.w0     =   % initial velocity along body z-axis
P.phi0   =   % initial roll angle
P.theta0 =   % initial pitch angle
P.psi0   =   % initial yaw angle
P.p0     =   % initial body frame roll rate
P.q0     =   % initial body frame pitch rate
P.r0     =   % initial body frame yaw rate

