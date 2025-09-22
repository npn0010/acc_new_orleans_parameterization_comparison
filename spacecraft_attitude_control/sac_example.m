function [obj,out] = sac_example(u_max,K_p,K_d)

% example 8.18 and 8.20 from Schaub and Junkins

J = diag([140; 100; 80]); % spacecraft inertia matrix

tf = 350; % maximum manuever time

sigma = [0.6; -0.4; 0.2]; % initial orientation in modified rodrigues parameters (MRP)
sigma_norm = sqrt(sigma.'*sigma); % norm of MRP

% convert to quaternions
q_0 = [0; 0; 0; 0]; 
q_0(1) = (1-sigma_norm^2)/(1+sigma_norm^2);
q_0(2:4) = 2*sigma/(1+sigma_norm^2);

w_0 = [0.70; 0.20; -0.15]; % initial angular velocity

x_0 = [q_0; w_0];

ode_opts = odeset('RelTol',1e-10,'AbsTol',1e-10,'Events',@(t,X) sac_event_function(t,X));

[t,x,~,~,ie]= ode113(@(t,x) sac_eom(t,x,u_max,J,K_p,K_d),[0 tf],[x_0; 0],ode_opts);

% check if it converged
if ie == 1
    obj = -1/x(end,8); % return control effort if converged
else
    obj = sac_event_function(t(end),x(end,:)); % return final error if didn't converge
end

out = {t,x,u_max,J,K_p,K_d};

if nargout > 1
    sac_output(out{:});
end

end
