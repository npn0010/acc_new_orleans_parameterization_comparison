function [dxdt,u] = sac_eom(t,x,u_max,J,K_p,K_d)

q = x(1:4);
w = x(5:7);

u = -K_p*q(2:4) - K_d*w;  % control law (Schaub and Junkins Eq. ...)

if ~isempty(u_max)
    if abs(u(1)) > u_max
        u(1) = sign(u(1))*u_max;
    end
    
    if abs(u(2)) > u_max
        u(2) = sign(u(2))*u_max;
    end
    
    if abs(u(3)) > u_max
        u(3) = sign(u(3))*u_max;
    end
end

dw = J\(u - cross(w,J*w));

Omega = [   0 -w(1) -w(2) -w(3); 
         w(1)     0  w(3) -w(2);
         w(2) -w(3)     0  w(1);
         w(3)  w(2) -w(1)     0];

dq = 1/2*Omega*q;

dxdt = [dq; dw; 1/2*(u.'*u)];
   
end