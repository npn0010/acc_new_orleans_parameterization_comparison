function [obj,out] = sac_example(K_p,K_d)
out = [];

J = diag([10; 15; 20]); % moment of inertia

tf = 60;

psi_0   = deg2rad(60);
theta_0 = deg2rad(80);
phi_0   = deg2rad(-60); 
euler_0 = [psi_0,theta_0,phi_0];
q_0     = eul2quat(euler_0).';

w_0 = [0; 0; 0]; % rest-to-rest
% w_0 = [0.01; -0.01; 0.01]; % detumble

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
    
    n = length(K_p);  % Change this to your desired matrix size
    
    % Build the matrix string for LaTeX display
    matrixStr = '\left[\begin{array}{';
    for i = 1:n
        matrixStr = [matrixStr 'c'];  % Add column alignment (c for center alignment)
    end
    matrixStr = [matrixStr '}'];
    
    % Fill in the matrix elements into the LaTeX string
    for i = 1:n
        for j = 1:n
            matrixStr = [matrixStr sprintf('%g', K_p(i,j))];  % Format numbers to 2 decimal places
            if j < n
                matrixStr = [matrixStr ' & '];  % Add column separator if not the last element in the row
            end
        end
        if i < n
            matrixStr = [matrixStr ' \\\\ '];  % Add row separator if not the last row
        end
    end
    
    matrixStr = [matrixStr '\end{array}\right]'];
    
    matrixStr_K_p = matrixStr;
    
    n = length(K_d);  % Change this to your desired matrix size
    
    % Build the matrix string for LaTeX display
    matrixStr = '\left[\begin{array}{';
    for i = 1:n
        matrixStr = [matrixStr 'c'];  % Add column alignment (c for center alignment)
    end
    matrixStr = [matrixStr '}'];
    
    % Fill in the matrix elements into the LaTeX string
    for i = 1:n
        for j = 1:n
            matrixStr = [matrixStr sprintf('%g', K_d(i,j))];  % Format numbers to 2 decimal places
            if j < n
                matrixStr = [matrixStr ' & '];  % Add column separator if not the last element in the row
            end
        end
        if i < n
            matrixStr = [matrixStr ' \\\\ '];  % Add row separator if not the last row
        end
    end
    
    matrixStr = [matrixStr '\end{array}\right]'];
    
    matrixStr_K_d = matrixStr;
    
    figure('Name','Quaternions')
    hold on
    box on
    axis padded
    plot(t,q0,'-k','DisplayName','$q_0$')
    plot(t,q1,'-r','DisplayName','$q_1$')
    plot(t,q2,'-g','DisplayName','$q_2$')
    plot(t,q3,'-b','DisplayName','$q_3$')
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