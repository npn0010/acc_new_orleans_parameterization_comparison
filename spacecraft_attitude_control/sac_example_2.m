function [obj,out] = sac_example_2(K_p,K_d)
out = [];

J = diag([10; 15; 20]); % moment of inertia

tf = 100;

ode_opts = odeset('RelTol',1e-10,'AbsTol',1e-10,'Events',@(t,X) sac_event_function(t,X));

w_0 = [0; 0; 0]; % rest-to-rest

n_psi_0   = 6;
n_theta_0 = 6;
n_phi_0   = 6;

psi_0_vec   = deg2rad(linspace(10,350,n_psi_0));
theta_0_vec = deg2rad(linspace(10,350,n_theta_0));
phi_0_vec   = deg2rad(linspace(10,350,n_phi_0));

% n_psi_0   = 3;
% n_theta_0 = 3;
% n_phi_0   = 3;
% 
% psi_0_vec   = deg2rad([90 170 190 270]);
% theta_0_vec = deg2rad([90 170 190 270]);
% phi_0_vec   = deg2rad([90 170 190 270]);

obj = 0;
nc = 0;
for i = 1:n_psi_0
    for j = 1:n_theta_0
        for k = 1:n_phi_0

            psi_0   = psi_0_vec(i);
            theta_0 = theta_0_vec(j);
            phi_0   = phi_0_vec(k); 
            euler_0 = [psi_0,theta_0,phi_0];
            q_0     = eul2quat(euler_0).';

            X_0 = [q_0;w_0];
            [t,X,te,ye,ie] = ode113(@(t,X) sac_eom(t,X,J,K_p,K_d),[0 tf],[X_0; 0],ode_opts);
            if isempty(ie)
                nc = nc + 1;
            end
            obj = obj + X(end,end);

        end
    end
end

if nc ~= 0
    obj = nc;
else
    obj = -1/obj;
end

%% Outputs

if nargout > 1

    n_psi_0   = 10;
    n_theta_0 = 10;
    n_phi_0   = 10;
    
    psi_0_vec   = deg2rad(linspace(10,350,n_psi_0));
    theta_0_vec = deg2rad(linspace(10,350,n_theta_0));
    phi_0_vec   = deg2rad(linspace(10,350,n_phi_0));

    t_all  = cell(n_psi_0,n_theta_0,n_phi_0);
    X_all  = cell(n_psi_0,n_theta_0,n_phi_0);
    c_flag = ones(n_psi_0,n_theta_0,n_phi_0);

    figure
    hold on
    box on
    axis equal

    x_f = [0; 0; 1];

    plot3(x_f(1),x_f(2),x_f(3),'k.','MarkerSize',20)

    
    for i = 1:n_psi_0
        for j = 1:n_theta_0
            for k = 1:n_phi_0
    
                psi_0   = psi_0_vec(i);
                theta_0 = theta_0_vec(j);
                phi_0   = phi_0_vec(k); 
                euler_0 = [psi_0,theta_0,phi_0];
                q_0     = eul2quat(euler_0).';

                q0 = q_0(1);
                q1 = q_0(2);
                q2 = q_0(3);
                q3 = q_0(4);
                R = [1-2*(q2^2+q3^2) 2*(q1*q2+q0*q3) 2*(q1*q3-q0*q2);
                     2*(q1*q2-q0*q3) 1-2*(q1^2+q3^2) 2*(q2*q3+q0*q1);
                     2*(q1*q3+q0*q2) 2*(q2*q3-q0*q1) 1-2*(q1^2+q2^2)];
                x_0 = R*x_f;
    
                X_0 = [q_0;w_0];
                [t_temp,X_temp,te,ye,ie] = ode113(@(t,X) sac_eom(t,X,J,K_p,K_d),[0 tf],[X_0; 0],ode_opts);
                t_all{i,j,k} = t_temp;
                X_all{i,j,k} = X_temp;
                if isempty(ie)
                    c_flag(i,j,k) = 0;
                    plot3(x_0(1),x_0(2),x_0(3),'.r','MarkerSize',20)
                else
                    plot3(x_0(1),x_0(2),x_0(3),'.b','MarkerSize',20)
                end
            end
        end
    end


end


end