function [dXdt,tau] = sac_eom(t,X,J,K_p,K_d)

q = X(1:4);
w = X(5:7);
    
% Lyapunov-based Control Law
tau = -K_p * q(2:4) - K_d * w;  % Control torque

% tau_max = 1;
% 
% if abs(tau(1)) > tau_max
%     tau(1) = sign(tau(1))*tau_max;
% end
% 
% if abs(tau(2)) > tau_max
%     tau(2) = sign(tau(2))*tau_max;
% end
% 
% if abs(tau(3)) > tau_max
%     tau(3) = sign(tau(3))*tau_max;
% end
    
% Spacecraft dynamics (Euler's equations of motion)
dw = J\(tau - cross(w, J * w));  % Angular acceleration
    
% Quaternion kinematics (update quaternion)
Omega = [   0 -w(1) -w(2) -w(3); 
         w(1)     0  w(3) -w(2);
         w(2) -w(3)     0  w(1);
         w(3)  w(2) -w(1)     0];

% quaternions time derivative
dq = 1/2*Omega*q;

dXdt = [dq; dw; 1/2*(tau.'*tau)];
   
end