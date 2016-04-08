% path follow
%  - follow straight line path or orbit
%
% Modified:
%   3/25/2010  - RB
%   6/5/2010   - RB
%   11/08/2010 - RB
%   14/11/2014 - RWB
%
% input is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d   - desired airspeed
%   r      - inertial position of start of waypoint path
%   q      - unit vector that defines inertial direction of waypoint path
%   c      - center of orbit
%   rho    - radius of orbit
%   lambda - direction of orbit (+1 for CW, -1 for CCW)
%   xhat   - estimated MAV states (pn, pe, pd, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi)
%
% output is:
%  Va_c - airspeed command
%  h_c  - altitude command
%  chi_c - heading command
%  phi_ff - feed forward roll command
%
function out = path_follow(in,P)
  
  NN = 0;
  flag      = in(1+NN);
  Va_d      = in(2+NN);
  r_path    = [in(3+NN); in(4+NN); in(5+NN)];
  q_path    = [in(6+NN); in(7+NN); in(8+NN)];
  c_orbit   = [in(9+NN); in(10+NN); in(11+NN)];
  rho_orbit = in(12+NN);
  lam_orbit = in(13+NN);
  NN = NN + 13;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  Va        = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  phi       = in(7+NN);
  theta     = in(8+NN);
  chi       = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
   r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  NN = NN + 16;
  t         = in(1+NN);
  
%   persistent integrator;
%   persistent error_d1;
  
  switch flag,
      case 1, % follow straight line path specified by r and q
          
          qn = q_path(1);
          qe = q_path(2);
          qd = q_path(3);
          rn = r_path(1);
          re = r_path(2);
          rd = r_path(3);
          
          p = [pn; pe; -h];
          ki = [0; 0; -1];
          
          chi_q = atan2(qe,qn);
          while (chi_q-chi) < -pi, % unwrap chi_q, case 1
              chi_q = chi_q + 2*pi;
          end
          while (chi_q-chi) > pi, % unwrap chi_q, case 2
              chi_q = chi_q - 2*pi;
          end
          e_py = -sin(chi_q)*(pn-rn) + cos(chi_q)*(pe-re);
          ei_p = p - r_path;
          n = cross(q_path,ki)/norm(cross(q_path,ki));
          si = ei_p - dot(ei_p,n)*n;
          sn = si(1);
          se = si(2);
          
          chi_c = chi_q - P.chi_inf*2/pi*atan(P.K_path*e_py);
          h_c = -rd - sqrt(sn^2+se^2)*(qd/sqrt(qn^2+qe^2));
          phi_ff = 0;
           
      case 2, % follow orbit specified by c, rho, lam
          
          cn = c_orbit(1);
          ce = c_orbit(2);
          cd = c_orbit(3);
          
%           if t == 0,
%               integrator = 0;
%               error_d1 = 0;
%           end
          
          d = sqrt((pn-cn)^2 + (pe-ce)^2);
          error = d - rho_orbit;
 
          if abs(error) < rho_orbit/2,
              chi_o = atan2(pe-ce,pn-cn);
              while (chi_o-chi) < -pi, % unwrap chi_o, case 1
                  chi_o = chi_o + 2*pi;
              end
              while (chi_o-chi) > pi, % unwrap chi_o, case 2
                  chi_o = chi_o - 2*pi;
              end
              
              % integrator to remove steady state error on orbit
              % - Ki_orbit chosen such that integrator saturation value
              % will add a maximum of 40deg to chi_c
%               if abs(error) < 50,
%                   integrator = integrator + P.Ts/2*(error + error_d1);
%                   if integrator > 1,
%                       integrator = 1;
%                   elseif integrator < -1,
%                       integrator = -1;
%                   end
%               else
%                   integrator = 0;
%               end
%               Ki_orbit = 0.2;

              chi_c = chi_o + lam_orbit*(pi/2 + atan(P.K_orbit*((d-rho_orbit)/rho_orbit)));% + Ki_orbit*integrator;
              h_c = -cd;
              phi_ff = lam_orbit*atan(Va^2/P.gravity/rho_orbit);
          else
              if error > 0,
                  chi_c = atan2(-(cn-pn),(ce-pe));
              else
                  chi_c = 0; %atan2(ce-pe,cn-pn);
              end
              h_c = -cd;
              phi_ff = 0; %atan(Va^2/P.gravity/rho_orbit);
          end
          
          error_d1 = error;
  end
  
  % command airspeed equal to desired airspeed
  Va_c = Va_d;
  
  % create output
  out = [Va_c; h_c; chi_c; phi_ff];
end


