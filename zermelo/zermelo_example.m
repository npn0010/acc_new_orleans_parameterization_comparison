function [obj,out] = zermelo_example(K)

% initial boundary conditions
X_0 = [-8; 6];

% maximum final time (infinite horizon problem so it's arbitrarily large)
tf = 1000;

% integration options
ode_options = odeset('RelTol',1e-10,'AbsTol',1e-10,'Events',@(t,X) zermelo_event_function(t,X));

% integrate dynamics with closed-loop controller
[t,X,te,ye,ie] = ode113(@(t,X) zermelo_eom(t,X,K),[0 tf],[X_0; 0],ode_options);

% evaluate cost
if ie == 1 % if solution converges according to event detection, return control effort
    obj = X(end,end);
else % return penalty if solution did not converge (it should always converge)
    obj = 1e8;
end

% post processing
if nargout > 1

    out.t = t;
    out.X = X;
    
    U    = zermelo_U(X.',K);
    V    = zermelo_V(X.',K);
    dVdt = zermelo_dVdt(X.',K);
    
    x = X(:,1);
    y = X(:,2);
    
    J = X(end,end);
    plot_title = sprintf('$J=%g$',J);
    
    matrixStr = sprintf('$K = \\left[\\begin{array}{cc}%.2f & %.2f \\\\ %.2f & %.2f\\end{array}\\right]$',K(1,1), K(1,2), K(2,1), K(2,2));
    
    xy_lb = -10;
    xy_ub = 10;
    
    fs = 16;

    figure('Name','Lyapunozermelo_Vtion')
    subplot(2,1,1)
    hold on
    box on
    plot(t,V,'-k','DisplayName','$V(t)$')
    xlabel('$t$','Interpreter','latex')
    ylabel('$V$','Interpreter','latex')
    legend('Interpreter','latex')
    set(gca,'TickLabelInterpreter','latex','FontSize',fs)
    
    subplot(2,1,2)
    hold on
    box on
    plot(t,dVdt,'-k','DisplayName','$\dot{V}(t)$')
    xlabel('$t$','Interpreter','latex')
    ylabel('$\dot{V}$','Interpreter','latex')
    legend('Interpreter','latex')
    set(gca,'TickLabelInterpreter','latex','FontSize',fs)
    sgtitle(plot_title,'Interpreter','latex')
    
    figure('Name','States')
    hold on
    box on
    plot(t,x,'-r','DisplayName','$x(t)$')
    plot(t,y,'-b','DisplayName','$y(t)$')
    xlabel('$t$','Interpreter','latex')
    ylabel('$\boldmath{X}$','Interpreter','latex')
    legend('Interpreter','latex')
    set(gca,'TickLabelInterpreter','latex','FontSize',fs)
    sgtitle(plot_title,'Interpreter','latex')
    
    figure('Name','Control')
    hold on
    box on
    plot(t,U(1,:),'-r','DisplayName','$u^*_x(t)$')
    plot(t,U(2,:),'-b','DisplayName','$u^*_y(t)$')
    xlabel('$t$','Interpreter','latex')
    ylabel('$\boldmath{u}^*$','Interpreter','latex')
    legend('Interpreter','latex')
    set(gca,'TickLabelInterpreter','latex','FontSize',fs)
    sgtitle(plot_title,'Interpreter','latex')

    xy_space = linspace(xy_lb,xy_ub,40);
    [X_grid,Y_grid] = meshgrid(xy_space,xy_space);
    
    figure('Name','Phase_Space')
    hold on
    box on
    quiver(X_grid,Y_grid,zermelo_f(X_grid,Y_grid),zermelo_g(X_grid,Y_grid),1,'DisplayName','$(f,g)$','LineWidth',1,'AutoScale','on','Color','k')
    plot(x,y,'-g','DisplayName',['$(x(t),y(t))$, ',matrixStr],'LineWidth',2)
    plot(0,0,'.b','MarkerSize',20,'DisplayName','$(x_0,y_0)$')
    plot(X_0(1),X_0(2),'.r','MarkerSize',20,'DisplayName','$(x_f,y_f)$')
    xlim([xy_lb xy_ub])
    ylim([xy_lb xy_ub])
    xlabel('$x$','Interpreter','latex')
    ylabel('$y$','Interpreter','latex')
    legend('Interpreter','latex','Location','southwest','Orientation','vertical')
    set(gca,'TickLabelInterpreter','latex','FontSize',fs)
    sgtitle(plot_title,'Interpreter','latex')
    
    K_flag = K(1,2) ~= 0;

    figure('Name','Lyapunozermelo_Vtion_3D')
    hold on
    box on
    % plot3(X(:,1),X(:,2),zermelo_V(X(:,1:2).',K),'-k','DisplayName','{\boldmath$x$}','LineWidth',2)
    plot3(x,y,zermelo_V([x y].',K),'-g','DisplayName',['$(x(t),y(t))$, ',matrixStr],'LineWidth',2)
    plot3(X_0(1),X_0(2),zermelo_V(X_0,K),'.r','MarkerSize',20,'DisplayName',sprintf('{\\boldmath$x$}$(0)=(%g,%g)$',X_0(1),X_0(2)))
    plot3(0,0,zermelo_V([0; 0],K),'.b','MarkerSize',20,'DisplayName','$(0,0)$')
    fmesh(@(x,y) zermelo_V([x;y],K),[xy_lb xy_ub xy_lb xy_ub],'ShowContours','off','MeshDensity',50,'DisplayName',sprintf('$V_%g(x,y)$',K_flag))
    % fmesh(@(x,y) zermelo_V([x;y],K),[xy_lb xy_ub xy_lb xy_ub],'ShowContours','on','MeshDensity',50,'DisplayName','$V(x,y)$')
    xlabel('$x$','Interpreter','latex')
    ylabel('$y$','Interpreter','latex')
    zlabel(sprintf('$V_%g$',K_flag),'Interpreter','latex')
    legend('Interpreter','latex','Location','best','Orientation','vertical')
    set(gca,'TickLabelInterpreter','latex','FontSize',16)
    if K_flag == 1
        view(20,45)
    elseif K_flag == 2
        view(30,25)
    end
    
    figure('Name','Lyapunov_Time_Derivative_3D')
    hold on
    box on
    plot3(X(:,1),X(:,2),zermelo_dVdt(X(:,1:2).',K),'-g','DisplayName',['$(x(t),y(t))$, ',matrixStr],'LineWidth',2)
    % plot3(X(:,1),X(:,2),zermelo_dVdt(X(:,1:2).',K),'-k','DisplayName','{\boldmath$x$}','LineWidth',2)
    plot3(X_0(1),X_0(2),zermelo_dVdt(X_0,K),'.r','MarkerSize',20,'DisplayName',sprintf('{\\boldmath$x$}$(0)=(%g,%g)$',X_0(1),X_0(2)))
    plot3(0,0,zermelo_dVdt([0;0],K),'.b','MarkerSize',20,'DisplayName','$(0,0)$')
    fmesh(@(x,y) zermelo_dVdt([x;y],K),[xy_lb xy_ub xy_lb xy_ub],'ShowContours','off','MeshDensity',50,'DisplayName',sprintf('$\\dot{V}_%g(x,y)$',K_flag))
    xlabel('$x$','Interpreter','latex')
    ylabel('$y$','Interpreter','latex')
    zlabel(sprintf('$\\dot{V}_%g$',K_flag),'Interpreter','latex')
    legend('Interpreter','latex','Location','best','Orientation','vertical')
    set(gca,'TickLabelInterpreter','latex','FontSize',16)
    if K_flag == 1
        view(-30,45)
    elseif K_flag == 2
        view(-100,20)
    end

    figure('Name','Control_Surface')
    hold on
    box on
    fmesh(@(x,y) zermelo_U_norm([x;y],K),[xy_lb xy_ub xy_lb xy_ub],'ShowContours','off','MeshDensity',50,'DisplayName',sprintf('$\\|${\\boldmath$u$}$_%g\\|$',K_flag))
    plot3(X_0(1),X_0(2),zermelo_U_norm(X_0,K),'.r','MarkerSize',20,'DisplayName',sprintf('{\\boldmath$x$}$(0)=(%g,%g)$',X_0(1),X_0(2)))
    plot3(0,0,zermelo_U([0; 0],K),'.b','MarkerSize',20,'DisplayName','$(0,0)$')
    plot3(X(:,1),X(:,2),zermelo_U_norm(X(:,1:2).',K),'-k','DisplayName','{\boldmath$x$}','LineWidth',2)
    % fmesh(@(x,y) zermelo_V([x;y],K),[xy_lb xy_ub xy_lb xy_ub],'ShowContours','on','MeshDensity',50,'DisplayName','$V(x,y)$')
    xlabel('$x$','Interpreter','latex')
    ylabel('$y$','Interpreter','latex')
    zlabel(sprintf('$\\|${\\boldmath$u$}$_%g\\|$',K_flag),'Interpreter','latex')
    legend('Interpreter','latex','Location','best','Orientation','vertical')
    set(gca,'TickLabelInterpreter','latex','FontSize',16)
    view(50,55)

end

end