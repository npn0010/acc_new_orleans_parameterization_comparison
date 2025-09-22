clc,clear,close all

%%

load sac_uc_K_1_pso_runs_10_18_Sep_2025_16_29_44.mat
obj_uc_K_1 = -1./obj;

load sac_uc_K_2_pso_runs_10_18_Sep_2025_16_35_17.mat
obj_uc_K_2 = -1./obj;

load sac_uc_K_3_pso_runs_10_18_Sep_2025_16_42_13.mat
obj_uc_K_3 = -1./obj;

load sac_c_K_1_pso_runs_10_18_Sep_2025_15_58_31.mat
obj_c_K_1 = -1./obj;

load sac_c_K_2_pso_runs_10_18_Sep_2025_16_12_16.mat
obj_c_K_2 = -1./obj;

load sac_c_K_3_pso_runs_10_18_Sep_2025_16_23_24.mat
obj_c_K_3 = -1./obj;

%%

figure
hold on
box on
axis padded
grid on
plot(1*ones(10,1),obj_uc_K_1,'.b','MarkerSize',20)
plot(2*ones(10,1),obj_uc_K_2,'.r','MarkerSize',20)
plot(3*ones(10,1),obj_uc_K_3,'.g','MarkerSize',20)
xticks([1 2 3])
xticklabels({'Diagonal','GR','CT'})
ylabel('$J$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16)

exportgraphics(gcf,'unconstrained_results.pdf','ContentType','vector')

figure
hold on
box on
axis padded
grid on
plot(1*ones(10,1),obj_c_K_1,'.b','MarkerSize',20)
plot(2*ones(10,1),obj_c_K_2,'.r','MarkerSize',20)
plot(3*ones(10,1),obj_c_K_3,'.g','MarkerSize',20)
xticks([1 2 3])
xticklabels({'Diagonal','GR','CT'})
ylabel('$J$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16)

exportgraphics(gcf,'constrained_results.pdf','ContentType','vector')

