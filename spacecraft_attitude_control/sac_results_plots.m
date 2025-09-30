clc,clear,close all

%% Plot Settings 

legend_fs = 10;
fs = 14;
save_flag = 1;

scriptPath = mfilename('fullpath');
scriptPath = fileparts(scriptPath);

%% Load Unconstrained Results

load sac_unconstrained_sol.mat

%% Get Best Unconstrained Solution

[~,I] = min(obj,[],'all','linear');
[I1,I2,I3] = ind2sub(size(obj),I);

K_flag = I1;
k_best = k_sol{I1,I2,I3};

u_max = [];
tf = 350; % maximum manuever time
your_func = @(K_p,K_d) sac_example(u_max,tf,K_p,K_d);

[obj_temp,K] = obj_func(k_best,your_func,K_flag,l,m,n);

[~,out] = your_func(K{:});

t_best_uc = out{1};
x_best_uc = out{2};
u_best_uc = out{3};

%% Get Best Unconstrained Diagonal K Solution

[~,I] = min(obj(1,:,:),[],'all','linear');
[I1,I2,I3] = ind2sub(size(obj(1,:,:)),I);

K_flag = I1;
k_best = k_sol{I1,I2,I3};

[obj_temp,K] = obj_func(k_best,your_func,K_flag,l,m,n);

[~,out] = your_func(K{:});

t_best_diagonal_uc = out{1};
x_best_diagonal_uc = out{2};
u_best_diagonal_uc = out{3};

%% Load Constrained Results

load sac_constrained_sol.mat

%% Get Best Constrained Solution

[~,I] = min(obj,[],'all','linear');
[I1,I2,I3] = ind2sub(size(obj),I);

K_flag = I1;
k_best = k_sol{I1,I2,I3};

u_max = 1;
tf = 350; % maximum manuever time
your_func = @(K_p,K_d) sac_example(u_max,tf,K_p,K_d);

[obj_temp,K] = obj_func(k_best,your_func,K_flag,l,m,n);

[~,out] = your_func(K{:});

t_best_c = out{1};
x_best_c = out{2};
u_best_c = out{3};

%% Get Best Constrained Diagonal K Solution

[~,I] = min(obj(1,:,:),[],'all','linear');
[I1,I2,I3] = ind2sub(size(obj(1,:,:)),I);

K_flag = I1;
k_best = k_sol{I1,I2,I3};

[obj_temp,K] = obj_func(k_best,your_func,K_flag,l,m,n);

[~,out] = your_func(K{:});

t_best_diagonal_c = out{1};
x_best_diagonal_c = out{2};
u_best_diagonal_c = out{3};

%%

close all

%% Plot Unconstrained Orientation

figure('Name','unconstrained_orientation')

subplot(2,2,1)
hold on
box on
axis padded
plot(t_best_uc,x_best_uc(:,1),'-k','DisplayName',['Best Full {\boldmath$K$} Solution ($J=$',num2str(x_best_uc(end,end)),')'])
plot(t_best_diagonal_uc,x_best_diagonal_uc(:,1),'-r','DisplayName',['Best Diagonal {\boldmath$K$} Solution ($J=$',num2str(x_best_diagonal_uc(end,end)),')'])
xlabel('Time [s]','Interpreter','latex')
ylabel('$q_0$','Interpreter','latex')
legend('Interpreter','latex','FontSize',legend_fs,'Orientation','vertical','Location','northoutside')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,2)
hold on
box on
axis padded
plot(t_best_uc,x_best_uc(:,2),'-k')
plot(t_best_diagonal_uc,x_best_diagonal_uc(:,2),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$q_1$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,3)
hold on
box on
axis padded
plot(t_best_uc,x_best_uc(:,3),'-k')
plot(t_best_diagonal_uc,x_best_diagonal_uc(:,3),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$q_2$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,4)
hold on
box on
axis padded
plot(t_best_uc,x_best_uc(:,4),'-k')
plot(t_best_diagonal_uc,x_best_diagonal_uc(:,4),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$q_3$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

% sgtitle('With Unconstrained Control','Interpreter','latex')

if save_flag
    exportgraphics(gcf,[scriptPath,filesep,'figures',filesep,gcf().Name,'.pdf'],'ContentType','vector')
end

%% Plot Unconstrained Angular Velocity

figure('Name','unconstrained_angular_velocity')

subplot(3,1,1)
hold on
box on
axis padded
plot(t_best_uc,x_best_uc(:,5),'-k','DisplayName',['Best Full {\boldmath$K$} Solution ($J=$',num2str(x_best_uc(end,end)),')'])
plot(t_best_diagonal_uc,x_best_diagonal_uc(:,5),'-r','DisplayName',['Best Diagonal {\boldmath$K$} Solution ($J=$',num2str(x_best_diagonal_uc(end,end)),')'])
xlabel('Time [s]','Interpreter','latex')
ylabel('$\omega_1$','Interpreter','latex')
legend('Interpreter','latex','FontSize',legend_fs,'Orientation','horizontal','Location','northoutside')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(3,1,2)
hold on
box on
axis padded
plot(t_best_uc,x_best_uc(:,6),'-k')
plot(t_best_diagonal_uc,x_best_diagonal_uc(:,6),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$\omega_2$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(3,1,3)
hold on
box on
axis padded
plot(t_best_uc,x_best_uc(:,7),'-k')
plot(t_best_diagonal_uc,x_best_diagonal_uc(:,7),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$\omega_3$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

% sgtitle('With Unconstrained Control','Interpreter','latex')

if save_flag
    exportgraphics(gcf,[scriptPath,filesep,'figures',filesep,gcf().Name,'.pdf'],'ContentType','vector')
end

%% Plot Unconstrained Control

figure('Name','unconstrained_control')

subplot(2,2,1)
hold on
box on
axis padded
plot(t_best_uc,x_best_uc(:,end),'-k','DisplayName',['Best Full {\boldmath$K$} Solution ($J=$',num2str(x_best_uc(end,end)),')'])
plot(t_best_diagonal_uc,x_best_diagonal_uc(:,end),'-r','DisplayName',['Best Diagonal {\boldmath$K$} Solution ($J=$',num2str(x_best_diagonal_uc(end,end)),')'])
xlabel('Time [s]','Interpreter','latex')
ylabel('$J$','Interpreter','latex')
legend('Interpreter','latex','FontSize',legend_fs,'Orientation','vertical','Location','northoutside')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,2)
hold on
box on
axis padded
plot(t_best_uc,u_best_uc(:,1),'-k')
plot(t_best_diagonal_uc,u_best_diagonal_uc(:,1),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$u_1$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,3)
hold on
box on
axis padded
plot(t_best_uc,u_best_uc(:,2),'-k')
plot(t_best_diagonal_uc,u_best_diagonal_uc(:,2),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$u_2$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,4)
hold on
box on
axis padded
plot(t_best_uc,u_best_uc(:,3),'-k')
plot(t_best_diagonal_uc,u_best_diagonal_uc(:,3),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$u_3$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

% sgtitle('With Unconstrained Control','Interpreter','latex')

if save_flag
    exportgraphics(gcf,[scriptPath,filesep,'figures',filesep,gcf().Name,'.pdf'],'ContentType','vector')
end

%% Plot Constrained Orientation

figure('Name','constrained_orientation')

subplot(2,2,1)
hold on
box on
axis padded
plot(t_best_c,x_best_c(:,1),'-k','DisplayName',['Best Full {\boldmath$K$} Solution ($J=$',num2str(x_best_c(end,end)),')'])
plot(t_best_diagonal_c,x_best_diagonal_c(:,1),'-r','DisplayName',['Best Diagonal {\boldmath$K$} Solution ($J=$',num2str(x_best_diagonal_c(end,end)),')'])
xlabel('Time [s]','Interpreter','latex')
ylabel('$q_0$','Interpreter','latex')
legend('Interpreter','latex','FontSize',legend_fs,'Orientation','vertical','Location','northoutside')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,2)
hold on
box on
axis padded
plot(t_best_c,x_best_c(:,2),'-k')
plot(t_best_diagonal_c,x_best_diagonal_c(:,2),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$q_1$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,3)
hold on
box on
axis padded
plot(t_best_c,x_best_c(:,3),'-k')
plot(t_best_diagonal_c,x_best_diagonal_c(:,3),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$q_2$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,4)
hold on
box on
axis padded
plot(t_best_c,x_best_c(:,4),'-k')
plot(t_best_diagonal_c,x_best_diagonal_c(:,4),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$q_3$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

% sgtitle('With Constrained Control','Interpreter','latex')

if save_flag
    exportgraphics(gcf,[scriptPath,filesep,'figures',filesep,gcf().Name,'.pdf'],'ContentType','vector')
end

%% Plot Constrained Angular Velocity

figure('Name','constrained_angular_velocity')

subplot(3,1,1)
hold on
box on
axis padded
plot(t_best_c,x_best_c(:,5),'-k','DisplayName',['Best Full {\boldmath$K$} Solution ($J=$',num2str(x_best_c(end,end)),')'])
plot(t_best_diagonal_c,x_best_diagonal_c(:,5),'-r','DisplayName',['Best Diagonal {\boldmath$K$} Solution ($J=$',num2str(x_best_diagonal_c(end,end)),')'])
xlabel('Time [s]','Interpreter','latex')
ylabel('$\omega_1$','Interpreter','latex')
legend('Interpreter','latex','FontSize',legend_fs,'Orientation','horizontal','Location','northoutside')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(3,1,2)
hold on
box on
axis padded
plot(t_best_c,x_best_c(:,6),'-k')
plot(t_best_diagonal_c,x_best_diagonal_c(:,6),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$\omega_2$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(3,1,3)
hold on
box on
axis padded
plot(t_best_c,x_best_c(:,7),'-k')
plot(t_best_diagonal_c,x_best_diagonal_c(:,7),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$\omega_3$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

% sgtitle('With Constrained Control','Interpreter','latex')

if save_flag
    exportgraphics(gcf,[scriptPath,filesep,'figures',filesep,gcf().Name,'.pdf'],'ContentType','vector')
end

%% Plot Constrained Control

figure('Name','constrained_control')

subplot(2,2,1)
hold on
box on
axis padded
plot(t_best_c,x_best_c(:,end),'-k','DisplayName',['Best Full {\boldmath$K$} Solution ($J=$',num2str(x_best_c(end,end)),')'])
plot(t_best_diagonal_c,x_best_diagonal_c(:,end),'-r','DisplayName',['Best Diagonal {\boldmath$K$} Solution ($J=$',num2str(x_best_diagonal_c(end,end)),')'])
xlabel('Time [s]','Interpreter','latex')
ylabel('$J$','Interpreter','latex')
legend('Interpreter','latex','FontSize',legend_fs,'Orientation','vertical','Location','northoutside')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,2)
hold on
box on
axis padded
plot(t_best_c,u_best_c(:,1),'-k')
plot(t_best_diagonal_c,u_best_diagonal_c(:,1),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$u_1$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,3)
hold on
box on
axis padded
plot(t_best_c,u_best_c(:,2),'-k')
plot(t_best_diagonal_c,u_best_diagonal_c(:,2),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$u_2$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

subplot(2,2,4)
hold on
box on
axis padded
plot(t_best_c,u_best_c(:,3),'-k')
plot(t_best_diagonal_c,u_best_diagonal_c(:,3),'-r')
xlabel('Time [s]','Interpreter','latex')
ylabel('$u_3$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',fs)

% sgtitle('With Constrained Control','Interpreter','latex')

if save_flag
    exportgraphics(gcf,[scriptPath,filesep,'figures',filesep,gcf().Name,'.pdf'],'ContentType','vector')
end