function [obj,out] = sac_example(K_p,K_d)
out = [];

J = diag([140; 100; 80]);

tf = 350;

% example 8.18 from Schaub and Junkins

sigma = [0.6; -0.4; 0.2]; % modified rodrigues parameters
sigma_norm = sqrt(sigma.'*sigma);

% convert to quaternions
q_0 = [0; 0; 0; 0]; 
q_0(1) = (1-sigma_norm^2)/(1+sigma_norm^2);
q_0(2:4) = 2*sigma/(1+sigma_norm^2);

w_0 = [0.70; 0.20; -0.15];

X_0 = [q_0;w_0];

ode_opts = odeset('RelTol',1e-10,'AbsTol',1e-10,'Events',@(t,X) sac_event_function(t,X));

[t,X,te,ye,ie]= ode113(@(t,X) sac_eom(t,X,J,K_p,K_d),[0 tf],[X_0; 0],ode_opts);

if ie == 1
    obj = -1/X(end,8);
else
    obj = 1e6;
end

%% Outputs

if nargout > 1

    legend_fs = 12;
    fs = 12;

    U_op = zeros(3,length(t));
    for i = 1:length(t)
        [~,U_op(:,i)] = sac_eom(t(i),X(i,:).',J,K_p,K_d);
    end

    q0 = X(:,1);
    q1 = X(:,2);
    q2 = X(:,3);
    q3 = X(:,4);
    wx = X(:,5);
    wy = X(:,6);
    wz = X(:,7);

    plot_title = sprintf('$J = %g$ [kg$\\cdot$m$^2$/s]',X(end,8));    
    
    figure('Name','Quaternions')
    hold on
    box on
    axis padded
    plot(t,q0,'-k','DisplayName','$q_0$')
    plot(t,q1,'-r','DisplayName','$q_1$')
    plot(t,q2,'-g','DisplayName','$q_2$')
    plot(t,q3,'-b','DisplayName','$q_3$')
    plot(t,vecnorm([q0 q1 q2 q3],2,2),'c-','DisplayName','$\|q\|$')
    xlabel('Time [s]','Interpreter','latex')
    ylabel('Quaternions [-]','Interpreter','latex')
    legend('Interpreter','latex','FontSize',legend_fs,'Orientation','horizontal','Location','best')
    set(gca,'TickLabelInterpreter','latex','FontSize',fs)
    sgtitle(plot_title,'Interpreter','latex')
    
    figure('Name','Angular_Velocity')
    hold on
    box on
    axis padded
    plot(t,rad2deg(wx),'-r','DisplayName','$\omega_x$')
    plot(t,rad2deg(wy),'-g','DisplayName','$\omega_y$')
    plot(t,rad2deg(wz),'-b','DisplayName','$\omega_z$')
    xlabel('Time [s]','Interpreter','latex')
    ylabel('Angular Velocity [deg/s]','Interpreter','latex')
    legend('Interpreter','latex','FontSize',legend_fs,'Orientation','horizontal','Location','best')
    set(gca,'TickLabelInterpreter','latex','FontSize',fs)
    sgtitle(plot_title,'Interpreter','latex')
    
    figure('Name','Control')
    hold on
    box on
    axis padded
    plot(t,U_op(1,:),'-r','DisplayName','$u_1$')
    plot(t,U_op(2,:),'-g','DisplayName','$u_2$')
    plot(t,U_op(3,:),'-b','DisplayName','$u_3$')
    xlabel('Time [s]','Interpreter','latex')
    ylabel('Control [N$\cdot$m]','Interpreter','latex')
    legend('Interpreter','latex','FontSize',legend_fs,'Orientation','horizontal','Location','best')
    set(gca,'TickLabelInterpreter','latex','FontSize',fs)
    sgtitle(plot_title,'Interpreter','latex')

end


end

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