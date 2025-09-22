function sac_output(t,x,u_max,J,K_p,K_d)

legend_fs = 12;
fs = 12;

u = zeros(3,length(t));
for i = 1:length(t)
    [~,u(:,i)] = sac_eom(t(i),x(i,:).',u_max,J,K_p,K_d);
end

q0 = x(:,1);
q1 = x(:,2);
q2 = x(:,3);
q3 = x(:,4);
w1 = x(:,5);
w2 = x(:,6);
w3 = x(:,7);

plot_title = sprintf('$J = %g$ [kg$\\cdot$m$^2$/s]',x(end,8));    

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
plot(t,rad2deg(w1),'-r','DisplayName','$\omega_x$')
plot(t,rad2deg(w2),'-g','DisplayName','$\omega_y$')
plot(t,rad2deg(w3),'-b','DisplayName','$\omega_z$')
xlabel('Time [s]','Interpreter','latex')
ylabel('Angular Velocity [deg/s]','Interpreter','latex')
legend('Interpreter','latex','FontSize',legend_fs,'Orientation','horizontal','Location','best')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)
sgtitle(plot_title,'Interpreter','latex')

figure('Name','Control')
hold on
box on
axis padded
plot(t,u(1,:),'-r','DisplayName','$u_1$')
plot(t,u(2,:),'-g','DisplayName','$u_2$')
plot(t,u(3,:),'-b','DisplayName','$u_3$')
xlabel('Time [s]','Interpreter','latex')
ylabel('Control [N$\cdot$m]','Interpreter','latex')
legend('Interpreter','latex','FontSize',legend_fs,'Orientation','horizontal','Location','best')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)
sgtitle(plot_title,'Interpreter','latex')


end