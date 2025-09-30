clc,clear,close all

%% Example Problems

matfilename = ['sac_sol_',char(datetime('now','Format','MM_dd_yyyy_HH_mm_ss')),'.mat'];

addpath('spacecraft_attitude_control')

u_max = []; % for UNCONSTRAINED contorl torque problem
% u_max = 1; % for CONSTRAINED control torque problem

tf = 350; % maximum manuever time

your_func = @(K_p,K_d) sac_example(u_max,tf,K_p,K_d);
m = [3 3];

initial_guess = [7.11 7.11 7.11 18.67 2.67 10.67]; % diagonal parameterization initial guess

lb_lambda = 1e-8;
ub_lambda = 20;

num_solver_runs = 20;

K_flag_vec = [1 2 3 4]; % 1 - diagonal, 2 - givens, 3 - cayley, 4 - cayley mapped

sol_method_vec = [1 2 3 5]; % 1 - ga, 2 - particleswarm, 3 - patternsearch, 4 - simulannealbnd, 5 - surrogateopt

use_parallel = false;

%%

ga_options             = optimoptions(            'ga','Display','iter','UseParallel',use_parallel,'PlotFcn','gaplotbestf');
particleswarm_options  = optimoptions( 'particleswarm','Display','iter','UseParallel',use_parallel,'PlotFcn','pswplotbestf');
patternsearch_options  = optimoptions( 'patternsearch','Display','iter','UseParallel',use_parallel,'PlotFcn','psplotbestf');
simulannealbnd_options = optimoptions('simulannealbnd','Display','iter','PlotFcn','saplotbestf');
surrogateopt_options   = optimoptions(  'surrogateopt','Display','iter','UseParallel',use_parallel,'PlotFcn','optimplotfvalconstr');

% initialize solution
k_sol     =  cell(4,5,num_solver_runs);
obj       =   inf(4,5,num_solver_runs);
out_info  =  cell(4,5,num_solver_runs);
comp_time = zeros(4,5,num_solver_runs);

l = length(m); % number of control matrices
n = (m.^2-m)./2; % determine number of orthogonal matrix parameterization variables needed

for K_flag = K_flag_vec

    % set bounds on eigenvalues of control matrices
    lb = lb_lambda*ones(sum(m),1);
    ub = ub_lambda*ones(sum(m),1);
    
    % set bounds on the orthogonal matrix parameterization variables
    if K_flag == 2
        lb = [lb; -pi*ones(sum(n),1)];
        ub = [ub;  pi*ones(sum(n),1)];
    elseif K_flag == 3
        lb = [lb; -100*ones(sum(n),1)];
        ub = [ub;  100*ones(sum(n),1)];
    elseif K_flag == 4
        lb = [lb; -(pi/2-0.01)*ones(sum(n),1)];
        ub = [ub;  (pi/2-0.01)*ones(sum(n),1)];
    end

    nvars = length(lb);
    
    fun = @(k) obj_func(k,your_func,K_flag,l,m,n);

    x0 = initial_guess(:);
    if K_flag > 1
        x0 = [x0; zeros(sum(n),1)];
    end

    for sol_method = sol_method_vec
        for i = 1:num_solver_runs
            tic
            if sol_method == 1
                [x,fval,~,output] = ga(fun,nvars,[],[],[],[],lb,ub,[],ga_options);
            elseif sol_method == 2
                [x,fval,~,output] = particleswarm(fun,nvars,lb,ub,particleswarm_options);
            elseif sol_method == 3
                [x,fval,~,output] = patternsearch(fun,x0,[],[],[],[],lb,ub,[],patternsearch_options);
            elseif sol_method == 4
                [x,fval,~,output] = simulannealbnd(fun,x0,lb,ub,simulannealbnd_options);
            elseif sol_method == 5
                [x,fval,~,output] = surrogateopt(fun,lb,ub,surrogateopt_options);
            end
            comp_time(K_flag,sol_method,i) = toc;
            k_sol{K_flag,sol_method,i} = x;
            obj(K_flag,sol_method,i) = fval;
            out_info{K_flag,sol_method,i} = output;
            if sol_method == 3
                break
            end
        end
    end
end

%%

[M,I] = min(obj,[],'all','linear');
[I1,I2,I3] = ind2sub(size(obj),I);

K_flag = I1;
k_best = k_sol{I1,I2,I3};

[~,K] = obj_func(k_best,your_func,I1,l,m,n);

[~,out] = your_func(K{:});

%%

save(['solutions',filesep,matfilename])

%%

figure('Name','all_obj_and_comp_time')
hold on
box on
grid minor
axis padded

color_vec = {'red','green','blue','cyan','magenta'};
marker_vec = {'o','square','diamond','^'};
color_legend_labels = {'GA','PSO','PS','SA','SO'};
marker_legend_labels = {'$K_\mathrm{D}$','$K_\mathrm{G}$','$K_\mathrm{C}$','$K_\mathrm{M}$'};

for i = K_flag_vec
    for j = sol_method_vec
        p1_temp = comp_time(i,j,:);
        p2_temp = -1./obj(i,j,:);
        p2_temp(obj(i,j,:) == inf) = nan;
        plot(p1_temp(:),p2_temp(:),'LineStyle','none','Marker',marker_vec{i},'Color',color_vec{j},'HandleVisibility','off','MarkerSize',10)
    end
end

k = 1;
for i = K_flag_vec
    for j = sol_method_vec
        if k == 1
            le_c{j} = plot(nan,'o','Color',color_vec{j},'MarkerFaceColor',color_vec{j},'DisplayName',color_legend_labels{j});
        end
    end
    k = k + 1;
    le_s{i} = plot(nan,'k','LineStyle','none','Marker',marker_vec{i},'DisplayName',marker_legend_labels{i});
end

xlabel('Computational Time [s]','Interpreter','latex')
ylabel('$J$','Interpreter','latex')

legend('Interpreter','latex','Orientation','horizontal')

set(gca,'FontSize',16,'TickLabelInterpreter','latex')

exportgraphics(gcf,[gcf().Name,'_',matfilename(1:end-4),'.pdf'],'ContentType','vector')
savefig(gcf,[gcf().Name,'_',matfilename(1:end-4)])

%%

figure('Name','sorted_by_parameterization_obj')
hold on
box on
grid minor
axis padded

k = 0;
for i = K_flag_vec
    k = k + 1;
    for j = sol_method_vec
        p1_temp = -1./obj(i,j,:);
        p1_temp(obj(i,j,:) == inf) = nan;
        plot(k,p1_temp(:),'LineStyle','none','Marker',marker_vec{i},'Color',color_vec{j},'HandleVisibility','off','MarkerSize',10)
    end
end

k = 1;
for i = K_flag_vec
    for j = sol_method_vec
        if k == 1
            le_c{j} = plot(nan,'o','Color',color_vec{j},'MarkerFaceColor',color_vec{j},'DisplayName',color_legend_labels{j});
        end
    end
    k = k + 1;
    le_s{i} = plot(nan,'k','LineStyle','none','Marker',marker_vec{i},'DisplayName',marker_legend_labels{i});
end

legend('Interpreter','latex','Orientation','horizontal')

xticks(1:k)
xticklabels(marker_legend_labels(K_flag_vec))
ylabel('$J$','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex','FontSize',16)

exportgraphics(gcf,[gcf().Name,'_',matfilename(1:end-4),'.pdf'],'ContentType','vector')
savefig(gcf,[gcf().Name,'_',matfilename(1:end-4)])
